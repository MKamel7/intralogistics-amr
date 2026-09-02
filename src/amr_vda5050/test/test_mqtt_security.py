#!/usr/bin/env python3
"""Nobody drives this vehicle over an anonymous plaintext connection.

The bridge turns an MQTT order into a NavigateToPose goal, so the question is
not academic: whoever can publish to the order topic can move a 250 kg vehicle
through a building with people in it. It previously connected anonymously, in
clear, to whatever host `broker_host` named.

These assertions need no broker, no ROS and no network, so they run in the fast
gate and fail the moment somebody restores the old default or writes a password
into the repository.
"""

import sys
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
from amr_vda5050.mqtt_security import (  # noqa: E402
    CA_ENV,
    DEFAULT_TLS_PORT,
    PASSWORD_ENV,
    PLAINTEXT_PORT,
    USER_ENV,
    Credentials,
    Tls,
    credentials_from_env,
    decide,
    describe,
    tls_from_env,
)

GOOD = Credentials("vehicle", "not-a-real-password")
TLS = Tls(ca="/etc/ssl/ca.crt")


def test_anonymous_is_refused():
    """The state the bridge shipped in."""
    decision = decide(Credentials(), Tls())

    assert decision.allowed is False
    assert USER_ENV in decision.reason


def test_half_a_credential_is_no_credential():
    assert decide(Credentials("vehicle", None), TLS).allowed is False
    assert decide(Credentials(None, "secret"), TLS).allowed is False


def test_a_password_without_tls_is_refused():
    """Because it would be shouted across the building in clear."""
    decision = decide(GOOD, Tls())

    assert decision.allowed is False
    assert CA_ENV in decision.reason


def test_credentials_over_tls_are_accepted_and_marked_secure():
    decision = decide(GOOD, TLS)

    assert decision.allowed is True
    assert decision.secure is True


def test_insecure_mode_exists_but_must_be_asked_for_by_name():
    """A local test broker is legitimate; a silent default is not."""
    assert decide(Credentials(), Tls(), allow_insecure=True).allowed is True
    assert decide(Credentials(), Tls(), allow_insecure=True).secure is False
    assert decide(Credentials(), Tls()).allowed is False


def test_insecure_mode_says_so_in_terms_that_cannot_be_skimmed_past():
    line = describe(decide(Credentials(), Tls(), allow_insecure=True), Tls(), PLAINTEXT_PORT)

    assert "INSECURELY" in line
    assert "drive this vehicle" in line


def test_the_default_port_is_the_encrypted_one():
    """A default that is insecure is the one most deployments keep."""
    assert DEFAULT_TLS_PORT == 8883
    assert PLAINTEXT_PORT == 1883


def test_a_certificate_that_is_not_there_does_not_count_as_encryption():
    """Reading as encrypted while not being encrypted is the whole failure."""
    tls = tls_from_env({CA_ENV: "/no/such/ca.crt"}, exists=lambda p: False)

    assert tls.enabled is False


def test_half_a_client_keypair_is_discarded_rather_than_half_used():
    tls = tls_from_env(
        {CA_ENV: "/ca", "AMR_VDA5050_CERT": "/cert"}, exists=lambda p: True)

    assert tls.enabled is True
    assert tls.mutual is False
    assert tls.cert is None


def test_credentials_come_from_the_environment_and_not_from_here():
    creds = credentials_from_env({USER_ENV: "vehicle", PASSWORD_ENV: "x"})

    assert creds.complete is True
    assert credentials_from_env({}).complete is False


def test_no_credential_is_written_down_in_this_package():
    """The check that survives somebody being helpful in a hurry."""
    source = (PKG / "amr_vda5050" / "mqtt_security.py").read_text(encoding="utf-8")
    bridge = (PKG / "amr_vda5050" / "vda5050_bridge.py").read_text(encoding="utf-8")

    for text in (source, bridge):
        lowered = text.lower()
        assert "password = \"" not in lowered
        assert "password='" not in lowered.replace("password=None", "")
        assert "passwd" not in lowered
