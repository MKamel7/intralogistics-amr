#!/usr/bin/env python3
"""Who may drive this vehicle over MQTT, and over what.

WHY THIS EXISTS

The bridge turns an MQTT message into a `NavigateToPose` goal. That is a remote
stranger moving a 250 kg vehicle around a building with people in it, and until
now it did so over an anonymous plaintext connection to whatever broker the
`broker_host` parameter named. Nothing failed when that was true, which is why
it stayed true, and it is the same shape as the OPC UA server in
`virtual-production-cell` that accepted anonymous writes to a safety object.

THE POLICY, AND WHY IT FAILS CLOSED

A vehicle interface with no credentials is not a vehicle interface, it is an
open door, so `decide()` refuses to connect rather than connecting insecurely.
The refusal is explicit and says what is missing, because the failure mode this
replaces was a bridge that came up looking healthy while accepting orders from
anybody.

There is one escape, `allow_insecure`, and it exists because a local test
broker on loopback is a legitimate thing to develop against. It must be asked
for by name, it is never the default, and the node says loudly when it is used.

NO CREDENTIAL IS WRITTEN DOWN HERE. Username and password come from the
environment, and the test asserts this file contains neither.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

USER_ENV = "AMR_VDA5050_USER"
PASSWORD_ENV = "AMR_VDA5050_PASSWORD"
CA_ENV = "AMR_VDA5050_CA"
CERT_ENV = "AMR_VDA5050_CERT"
KEY_ENV = "AMR_VDA5050_KEY"

#: The MQTT-over-TLS port. The bridge used to default to 1883, which is
#: plaintext, and a default that is insecure is the one most deployments keep.
DEFAULT_TLS_PORT = 8883
PLAINTEXT_PORT = 1883


@dataclass(frozen=True)
class Credentials:
    """The account the vehicle presents to the broker."""

    username: str | None = None
    password: str | None = None

    @property
    def complete(self) -> bool:
        return bool(self.username) and bool(self.password)


@dataclass(frozen=True)
class Tls:
    """The certificate authority, and optionally a client keypair."""

    ca: str | None = None
    cert: str | None = None
    key: str | None = None

    @property
    def enabled(self) -> bool:
        return bool(self.ca)

    @property
    def mutual(self) -> bool:
        """True when the vehicle also proves its own identity to the broker."""
        return bool(self.ca and self.cert and self.key)


@dataclass(frozen=True)
class Decision:
    """Connect or refuse, and the reason either way."""

    allowed: bool
    reason: str
    secure: bool = False


def credentials_from_env(environ=None) -> Credentials:
    environ = os.environ if environ is None else environ
    return Credentials(environ.get(USER_ENV) or None,
                       environ.get(PASSWORD_ENV) or None)


def tls_from_env(environ=None, exists=os.path.isfile) -> Tls:
    """The TLS material, keeping only paths that are actually there.

    A configured certificate that does not exist is worse than none: it reads
    as encrypted in the log and is not, which is the failure this whole module
    is about.
    """
    environ = os.environ if environ is None else environ
    ca = environ.get(CA_ENV)
    cert = environ.get(CERT_ENV)
    key = environ.get(KEY_ENV)
    ca = ca if ca and exists(ca) else None
    cert = cert if cert and exists(cert) else None
    key = key if key and exists(key) else None
    if not (cert and key):
        cert = key = None       # half a keypair is not a keypair
    return Tls(ca, cert, key)


def decide(credentials: Credentials, tls: Tls, *, allow_insecure: bool = False) -> Decision:
    """May the bridge connect, given what it was configured with?

    Both halves are required, and for different reasons. Credentials say WHO
    is allowed to command the vehicle; TLS stops anyone on the network reading
    or forging those commands in flight. A password over plaintext MQTT is a
    password shouted across the building.
    """
    if allow_insecure:
        return Decision(True, "insecure mode was explicitly requested", secure=False)
    if not credentials.complete:
        return Decision(
            False,
            f"no broker credentials: set {USER_ENV} and {PASSWORD_ENV}, or ask "
            f"for allow_insecure explicitly to use a local test broker")
    if not tls.enabled:
        return Decision(
            False,
            f"credentials without TLS would put the password on the wire in "
            f"clear: set {CA_ENV} to the broker's certificate authority, or ask "
            f"for allow_insecure explicitly")
    return Decision(True, "authenticated over TLS", secure=True)


def describe(decision: Decision, tls: Tls, port: int) -> str:
    """One line for the log that says what actually happened."""
    if not decision.allowed:
        return f"REFUSED to connect: {decision.reason}"
    if not decision.secure:
        return (f"connected INSECURELY on port {port}: anyone who can reach the "
                f"broker can drive this vehicle. {decision.reason}")
    return (f"connected on port {port}, authenticated over TLS"
            f"{' with a client certificate' if tls.mutual else ''}")
