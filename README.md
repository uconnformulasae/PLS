# Predictive LapSim: website

This branch is the lap-simulation website, published by GitHub Pages at
**https://uconnformulasae.github.io/PLS/**. It holds only the website. The simulation itself (the `lapsim`
package, the car files, the tracks, the tests) is on [`main`](https://github.com/uconnformulasae/PLS/tree/main).

```
index.html             the page
_config.yml            GitHub Pages settings
docs/website.md        how the site works: build, publishing, every page, pre-computed data
assets/
  css/  js/  img/      style sheet, scripts, logo
  py/                  the bridge to lapsim (lapsim_web.py), wing sections, the simulation bundle (lapsim.zip)
  data/                baseline results, drivers, pre-computed multibody and CFD runs, team designs (team/)
.github/
  build.py             builds assets/py/lapsim.zip and assets/data/ from a checkout of main
  workflows/build.yml  rebuilds after a change to the site
```

The bundle and the baseline results are rebuilt by GitHub Actions after every push to `main` that passes the
tests, and after every change to the site; nobody needs to rebuild by hand. To work on the site locally:

```bash
python .github/build.py --lapsim ../PLS-main --serve     # ../PLS-main: a checkout of main
```

See [docs/website.md](docs/website.md).
