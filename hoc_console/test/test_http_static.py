from pathlib import Path

from hoc_console.http_static import resolve_frontend_dist, resolve_frontend_source


def test_resolve_frontend_source_finds_package_json():
    path = resolve_frontend_source()
    assert path is not None
    assert (path / 'package.json').is_file()


def test_resolve_frontend_dist_finds_index_html():
    path = resolve_frontend_dist()
    assert path is not None
    assert (path / 'index.html').is_file()


def test_resolve_frontend_source_honors_env(monkeypatch, tmp_path):
    pkg = tmp_path / 'frontend'
    pkg.mkdir()
    (pkg / 'package.json').write_text('{}', encoding='utf-8')
    monkeypatch.setenv('HOC_FRONTEND_DIR', str(pkg))
    assert resolve_frontend_source() == pkg.resolve()
