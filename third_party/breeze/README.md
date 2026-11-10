# BreezeStyleSheets, light theme (vendored)

The Qt stylesheet of the studio's look (studio config UI design 3.5), from
[BreezeStyleSheets](https://github.com/Alexhuszagh/BreezeStyleSheets), MIT licence (`LICENSE`).

- Upstream tag `0.0.1` (the latest release), commit `5d3b3010ea70823fc22c1c28049ba9d889e1b680`.
- Built with the upstream `configure.py` (Python 3.12, stdlib only), in a clone outside this
  repository:

  ```sh
  git clone --branch 0.0.1 https://github.com/Alexhuszagh/BreezeStyleSheets src
  cd src
  python3 configure.py --styles=light-blue --resource breeze.qrc --output-dir ../out
  ```

  and `out/` copied here as it is. In this release the light theme is the `light-blue` style:
  `light-blue/` holds its stylesheet and icons, and `configure.py` copies the stylesheet to
  `light/stylesheet.qss`, which is the one the studio loads (`:/light/stylesheet.qss`). The
  icon URLs of the stylesheet name `:/light-blue/...`. `breeze.qrc` lists every file under the
  prefix `/`.

These files are generated: do not edit them. To update, regenerate from a newer tag with the same
command and replace the folder, then update the tag and commit above.
