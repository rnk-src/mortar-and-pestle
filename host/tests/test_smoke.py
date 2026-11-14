from importlib.metadata import version

import mortar_pestle


def test_package_version_matches_project_metadata() -> None:
    assert mortar_pestle.__version__ == version("mortar-pestle")
