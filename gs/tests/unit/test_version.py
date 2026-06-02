import fpvdgs


def test_version_is_a_string():
    assert isinstance(fpvdgs.__version__, str)
    assert fpvdgs.__version__
