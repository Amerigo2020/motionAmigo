import motionamigo


def test_version():
    assert motionamigo.__version__.count(".") == 2
