from hello_nidar import setup_message


def test_setup_message() -> None:
    assert setup_message() == "NIDAR setup is working."