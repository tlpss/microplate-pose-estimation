from microplate.dummy import dummy_func


def test_dummy():
    d = dummy_func()
    assert d
