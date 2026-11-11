"""Build the Loosey Goosey Robotics Arduino boards package (lgr/) and its Boards Manager index.

    python tools/arduino_package.py install    # copy the package into the sketchbook, to try it without a release
    python tools/arduino_package.py release    # write lgr-rp2040-<version>.zip and package_lgr_index.json into releases/

The package borrows the Raspberry Pi Pico core (earlephilhower/arduino-pico) through `build.core=rp2040:rp2040`, and
inherits that core's platform.txt: its build recipes, tools and upload. lgr/platform.txt only sets the name and
version, and boards.txt sets runtime.use_core_platform_path_for_runtime_platform_path so the inherited recipes find
their files in the core's folder. So the package builds with whichever core version the user has installed, from the
minimum the variant checks for. Users install the core from its own Boards Manager index; this index lists only this
package.

The package bundles the LGR library (lgr/libraries/LGR) and copies of Pico PIO USB and Adafruit NeoPixel from
the sketchbook's libraries folder at the versions in BUNDLED. TinyUSB comes with the core.

The index and the zips in releases/ are served from https://arduino.looseygooseyrobotics.com by GitHub Pages:
.github/workflows/pages.yml publishes the folder on every push to main that changes it. To publish a release: bump
VERSION here and in lgr/platform.txt, run `release`, then commit and push. The index keeps every earlier version, so a
release never removes one.
"""
import argparse
import hashlib
import json
import re
import shutil
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "lgr"
RELEASES = ROOT / "releases"
BUILD = ROOT / "build"

VERSION = "0.2.0"
BUNDLED = {"Pico_PIO_USB": "0.7.3", "Adafruit_NeoPixel": "1.15.5"}
SITE_URL = "https://arduino.looseygooseyrobotics.com"
WEBSITE = "https://looseygooseyrobotics.com"

SKETCHBOOK = Path.home() / "Documents/Arduino"


def stage(dest):
    """The package as it installs: lgr/ plus the bundled libraries."""
    if dest.exists():
        shutil.rmtree(dest)
    shutil.copytree(SRC, dest)
    for name, version in BUNDLED.items():
        lib = SKETCHBOOK / "libraries" / name
        props = (lib / "library.properties").read_text(encoding="utf-8")
        found = re.search(r"^version=(.*)$", props, re.M).group(1).strip()
        if found != version:
            sys.exit(f"{lib} is {found}; BUNDLED wants {version}")
        shutil.copytree(lib, dest / "libraries" / name, ignore=shutil.ignore_patterns(".git*", ".github"))


def install():
    dest = SKETCHBOOK / "hardware/lgr/rp2040"
    stage(dest)
    print(f"installed in {dest}; restart the IDE and pick Tools > Board > Loosey Goosey Robotics > goosebrain")


def release(out):
    platform_txt = (SRC / "platform.txt").read_text(encoding="utf-8")
    found = re.search(r"^version=(.*)$", platform_txt, re.M).group(1).strip()
    if found != VERSION:
        sys.exit(f"lgr/platform.txt says version={found}; VERSION is {VERSION}")
    name = f"lgr-rp2040-{VERSION}.zip"
    archive = out / name
    index_path = out / "package_lgr_index.json"
    if archive.exists():
        sys.exit(f"{archive} is already published; bump VERSION")
    previous = []
    if index_path.exists():
        old = json.loads(index_path.read_text(encoding="utf-8"))
        previous = next(p for p in old["packages"] if p["name"] == "lgr")["platforms"]
    staged = BUILD / "stage" / f"lgr-rp2040-{VERSION}"
    stage(staged)
    with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED) as z:
        for p in sorted(staged.rglob("*")):
            z.write(p, p.relative_to(staged.parent).as_posix())
    data = archive.read_bytes()

    index = {"packages": [{
        "name": "lgr",
        "maintainer": "Loosey Goosey Robotics",
        "websiteURL": WEBSITE,
        "email": "",
        "help": {"online": WEBSITE},
        "platforms": [{
            "name": "Loosey Goosey Robotics",
            "architecture": "rp2040",
            "version": VERSION,
            "category": "Contributed",
            "url": f"{SITE_URL}/{name}",
            "archiveFileName": name,
            "checksum": "SHA-256:" + hashlib.sha256(data).hexdigest(),
            "size": str(len(data)),
            "help": {"online": WEBSITE},
            "boards": [{"name": "goosebrain"}],
            # the compiler and upload tools come with the core, which the user installs
            "toolsDependencies": [],
        }] + [p for p in previous if p["version"] != VERSION],
        "tools": [],
    }]}
    index_path.write_text(json.dumps(index, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {archive} ({len(data)} bytes) and {index_path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("step", choices=["install", "release"])
    ap.add_argument("--out", type=Path, default=RELEASES, help="release: where to write (default releases/)")
    args = ap.parse_args()
    if args.step == "release":
        args.out.mkdir(parents=True, exist_ok=True)
        release(args.out)
    else:
        install()


if __name__ == "__main__":
    main()
