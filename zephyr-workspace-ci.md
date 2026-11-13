# Workspace pro lab image (ESPRESSObin, ROCK Pi 4): lokálně a v GitHub Actions s `zephyrproject-rtos/ci`

Popisuje, jak je postavený můj workspace a jak ho zreprodukovat, aby šly lab image stavět nezávisle: lokálně, v kontejneru nebo v GitHub Actions. Skripty a workflow jsou ve větvi `rk3399` (commity `b371c130d` a `e86ba3351`), která obsahuje i celou větev `espressobin`.

| Soubor | Co dělá |
|---|---|
| `scripts/ci/lab/common.sh` | Společné funkce: hostitelské balíčky, Zephyr SDK, west T2 workspace, firmware na SHA, README, SHA256SUMS |
| `scripts/ci/lab/espressobin.sh` | ESPRESSObin v5: netshell se všemi EP funkcemi, SATA, lab agent a LLEXT moduly → `-spi.bin`, `-sata.bin`, `-uart.tgz`, `*.llext` |
| `scripts/ci/lab/rock_pi_4.sh` | ROCK Pi 4: TF-A BL31, U-Boot TPL/SPL bez rkbin, `pcie_shell` → `-spi.bin`, `-sd.bin` |
| `.github/workflows/lab-images.yml` | Spouští skripty v kontejneru `ghcr.io/zephyrproject-rtos/ci:v0.29.4`, nahraje artefakty, volitelně i release |

Stejný skript běží beze změny na hostiteli, v kontejneru i v Actions. Rozdíl je jen v tom, odkud se vezme SDK a Python.

## 1. Jak vypadá můj workspace

| Co | Kde / verze |
|---|---|
| west topdir | `/root/work` (`.west/config`: `manifest.path=zephyr`) |
| Zephyr | `/root/work/zephyr-a3700`, worktree repozitáře `/root/work/zephyr`. Upstream základ `124121fbb`, větve `espressobin` a `rk3399`. Buildím s `ZEPHYR_BASE=$PWD`. |
| gh-pages | worktree `/root/work/ghp` |
| moduly | jen `cmsis`, `cmsis_6`, `fatfs`, `hal_ti`, `mbedtls`, `tf-psa-crypto` v `/root/work/modules`. `hal_ti` image nepotřebují. |
| Zephyr SDK | 1.0.1 minimal v `/root/sdk/zephyr-sdk-1.0.1`, toolchainy `aarch64-zephyr-elf` a `arm-zephyr-eabi`, `setup.sh -c` (CMake package) |
| hostitel | Ubuntu 24.04: cmake 3.28, ninja, dtc, Python 3.13, west 1.5.0 a requirements Zephyru přes `pip --break-system-packages` (bez venv) |
| Linux GCC | `gcc-aarch64-linux-gnu` 13.3 (TF-A, U-Boot), `gcc-arm-linux-gnueabi` (WTMI na CM3) |
| TF-A ESPRESSObin | `/root/work/tfa`, větev `espressobin-cci` `d2064baa8` (CCI SiP, BL33 load address) |
| TF-A RK3399 | worktree `/root/work/tfa-rk3399`, větev `rk3399-m0-ns` `e09b846ea` (SiP řízení M0, oprava `-T` pro M0 link) |
| A3700-utils | `/root/work/a3700-utils`, větev `espressobin-cm3` `fc61a21` (override `WTMI_EXEC_ADDR`) |
| mv-ddr | `/root/work/tfa/contrib/mv-ddr` `7bcb9dc` (upstream master) |
| Crypto++ | `/root/work/cryptopp` `5873a73a` (upstream master) |
| U-Boot | `/root/work/uboot`, větev `rk3399-perilp-m0` `ae095c51` (jeden commit nad upstream `65a75325`), stock `rock-pi-4-rk3399_defconfig` |
| sestavení ESPRESSObin 150 | `build-eb-all`: `netshell` se snippety `espressobin-{pcie-ep,pcie-nvme,sata,vntb,nvme-target,fabric,lab}`, `EXTRA_CONF_FILE=pcie-ep-uartlog.conf`, `NETSHELL_AUTHORIZED_KEY=…pem`, pak `mkflash.sh` s `BOOTDEV=SPINOR` a `SATA` |

Skripty dělají totéž, jen čistě a reprodukovatelně:

- Workspace je **west T2**. Fork Zephyru je sám manifest repozitářem (`west init -l`).
- Moduly se stahují přes `manifest.project-filter` jen ty potřebné, s `--depth=1`.
- Firmware okolo je připnuté na SHA v `$WS/fw/<jméno>`.

Rozložení workspace:

```
$WS/.west
$WS/zephyr            klon forku, větev rk3399 (nebo espressobin)
$WS/modules/...       cmsis, cmsis_6, fatfs, mbedtls, tf-psa-crypto
$WS/fw/tfa            TF-A espressobin-cci
$WS/fw/tfa-rk3399     TF-A rk3399-m0-ns
$WS/fw/a3700-utils, mv-ddr, cryptopp, u-boot
$WS/build/...         buildy
$WS/out               image, README.md, SHA256SUMS   (OUT=…)
```

## 2. Lokálně na hostiteli (Ubuntu 24.04 / Debian testing)

Hostitelské balíčky (jednou):

```sh
sudo apt-get install --no-install-recommends git cmake ninja-build gperf ccache \
  device-tree-compiler wget xz-utils file make gcc g++ python3-dev python3-venv \
  gcc-aarch64-linux-gnu gcc-arm-linux-gnueabi bc bison flex swig \
  libssl-dev libgnutls28-dev uuid-dev dosfstools mtools perl
```

Zephyr SDK 1.0.1. Stačí minimal a dva toolchainy, `-c` zaregistruje CMake package:

```sh
cd ~/sdk
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v1.0.1/zephyr-sdk-1.0.1_linux-x86_64_minimal.tar.xz
tar xf zephyr-sdk-1.0.1_linux-x86_64_minimal.tar.xz
zephyr-sdk-1.0.1/setup.sh -t aarch64-zephyr-elf -t arm-zephyr-eabi -c
```

Workspace a Python venv. West a requirements Zephyru patří do venv, systémový Python zůstane netknutý:

```sh
mkdir -p ~/lab && cd ~/lab
git clone -b rk3399 https://github.com/cz172638/zephyr zephyr
python3 -m venv .venv && . .venv/bin/activate
pip install west setuptools -r zephyr/scripts/requirements-base.txt   # setuptools: pylibfdt v U-Boot
```

Build:

```sh
. ~/lab/.venv/bin/activate
cd ~/lab
OUT=$PWD/out-rp4 zephyr/scripts/ci/lab/rock_pi_4.sh
NETSHELL_AUTHORIZED_KEY=~/netshell.pem OUT=$PWD/out-eb zephyr/scripts/ci/lab/espressobin.sh
```

Proměnné (všechny volitelné):

| Proměnná | Výchozí | K čemu |
|---|---|---|
| `WS` | rodič stromu Zephyru | west topdir |
| `OUT` | `$WS/out` | výstupy |
| `ZEPHYR_SDK_INSTALL_DIR` | hledá `/opt/toolchains`, `~/zephyr-sdk-*`, `~/sdk/zephyr-sdk-*` | SDK |
| `WEST_MODULES` | `cmsis,cmsis_6,fatfs,mbedtls,tf-psa-crypto` | west project filter |
| `IMG`, `NAME` | ESPRESSObin: `CONFIG_LAB_IMAGE` ze snippetu (150) a `lab-mods`; ROCK Pi 4: `152` a `storage` | jména souborů a prefix LLEXT |
| `CLOCKS`, `DDR_TOPOLOGY` | `CPU_800_DDR_800`, `2` | ESPRESSObin (C100 čip: `CPU_1000_DDR_800`) |
| `NETSHELL_AUTHORIZED_KEY` | povinné pro ESPRESSObin | PEM text nebo cesta |
| `TFA_URL`/`TFA_SHA`, `WTP_URL`/`WTP_SHA`, `MV_DDR_*`, `CRYPTOPP_*`, `UBOOT_URL`/`UBOOT_SHA` | forky `cz172638/…` a SHA z tabulky výše | firmware; jde i `file:///cesta/k/repu` |

**Číslo image:** `IMG` předává `-DCONFIG_LAB_IMAGE` a podle něj se pojmenují `.bin` i `.llext` a kontroluje verze při `lab mod load`. U nového obsahu zvyš číslo: `IMG=153 … espressobin.sh`, případně uprav snippet `espressobin-lab`.

Ruční build bez skriptu (stejné příkazy, jaké používá skript):

```sh
# ROCK Pi 4
west build -b rock_pi_4/rk3399/a53 zephyr/samples/boards/rockchip/rk3399/pcie_shell
zephyr/scripts/rockchip/rk3399-image.pl --bl31 fw/tfa-rk3399/build/rk3399/release/bl31/bl31.elf \
  --zephyr build/zephyr/zephyr.elf --uboot build/u-boot-rock-pi-4 -o 152-rp4-storage

# ESPRESSObin
west build -b espressobin/armada3720 zephyr/samples/boards/globalscale/espressobin/netshell \
  -S espressobin-pcie-ep -S espressobin-pcie-nvme -S espressobin-sata -S espressobin-vntb \
  -S espressobin-nvme-target -S espressobin-fabric -S espressobin-lab -- \
  -DEXTRA_CONF_FILE=pcie-ep-uartlog.conf -DNETSHELL_AUTHORIZED_KEY=$HOME/netshell.pem
west build -t lab_mods                       # kontrola symbolů LLEXT proti firmwaru
TFA=… WTP=… MV_DDR=… CRYPTOPP=… BOOTDEV=SPINOR \
  sh zephyr/boards/globalscale/espressobin/support/mkflash.sh build/zephyr/zephyr.bin
```

## 3. Lokálně v kontejneru (jako v Actions)

Docker i podman (rootless). Uvnitř kontejneru běží skript jako root a doinstaluje si chybějící balíčky:

```sh
cd ~/lab
podman run --rm -it -v "$PWD":/ws:Z -w /ws \
  -e WS=/ws -e OUT=/ws/out-rp4 \
  ghcr.io/zephyrproject-rtos/ci:v0.29.4 \
  bash -eu zephyr/scripts/ci/lab/rock_pi_4.sh
```

Co obraz `ci:v0.29.4` má a co si skript doinstaluje:

| Má | Nemá → `apt-get install` ve skriptu |
|---|---|
| Zephyr SDK 1.0.1 (všechny toolchainy) v `/opt/toolchains/zephyr-sdk-1.0.1` | `gcc-aarch64-linux-gnu`, `gcc-arm-linux-gnueabi` |
| venv `/opt/python/venv` s requirements Zephyru (west, pyelftools, …) | `swig`, `libgnutls28-dev`, `uuid-dev` (U-Boot tools) |
| cmake, ninja, dtc, gcc/g++, bison, flex, libssl-dev, python3-dev, perl, git | `bc`, `dosfstools`, `mtools` (FAT na SD image) |

Tag `v0.29.4` odpovídá `ci-repo-cache:v0.29.4.*` v `.github/workflows/twister.yaml` stromu, ze kterého větev vychází, a SDK 1.0.1, se kterým buildím. Při rebase na novější upstream zvedni tag podle `twister.yaml`.

## 4. GitHub Actions

### Jednorázově

1. **Větev Zephyru do forku:**

   ```sh
   git fetch ../zephyr-espressobin-inc.bundle rk3399:rk3399 espressobin:espressobin gh-pages:gh-pages
   git push origin rk3399 espressobin gh-pages
   ```

2. **Firmware větve do forků.** Skripty je berou z `cz172638/…` na pevných SHA, přes `git fetch <sha>`. Proto musí být commity na GitHubu.

   ```sh
   # TF-A: espressobin-cci + rk3399-m0-ns (bundle potřebuje upstream eb60949df a 0892ad4db)
   git -C arm-trusted-firmware fetch https://github.com/ARM-software/arm-trusted-firmware master
   git -C arm-trusted-firmware fetch ../tf-a-lab.bundle espressobin-cci:espressobin-cci rk3399-m0-ns:rk3399-m0-ns
   git -C arm-trusted-firmware push origin espressobin-cci rk3399-m0-ns
   # U-Boot: rk3399-perilp-m0 (nad upstream main 65a75325)
   git -C u-boot fetch ../u-boot-lab.bundle rk3399-perilp-m0:rk3399-perilp-m0
   git -C u-boot push origin rk3399-perilp-m0
   # A3700-utils: založ fork cz172638/A3700-utils-marvell, pak
   git clone https://github.com/cz172638/A3700-utils-marvell && cd A3700-utils-marvell
   git fetch ../a3700-utils-lab.bundle espressobin-cm3:espressobin-cm3 && git push origin espressobin-cm3
   ```

   Forky nech veřejné. U privátních dej do repa secret s PAT a do `*_URL` adresu s tokenem, protože `GITHUB_TOKEN` jiná repa nevidí.

3. **Klíč pro netshell** jako proměnnou repozitáře (veřejný klíč, ne secret):

   ```sh
   ssh-keygen -e -m PKCS8 -f ~/.ssh/id_rsa.pub > netshell.pem
   gh variable set NETSHELL_AUTHORIZED_KEY -R cz172638/zephyr < netshell.pem
   ```

4. **Spouštění.** Push do `rk3399` nebo `espressobin`, který mění zdrojáky (seznam `paths` ve workflow), spustí obě desky.
   - Ruční `workflow_dispatch` vidí GitHub jen u workflow na **výchozí větvi** (u tebe `ci`). Buď zkopíruj `lab-images.yml` i do `ci`, nebo spouštěj pushem.

   ```sh
   gh workflow run lab-images.yml -R cz172638/zephyr --ref rk3399 -f boards=rock_pi_4 -f release=true
   gh run watch -R cz172638/zephyr
   gh run download -R cz172638/zephyr -n rock_pi_4-images -D rp4 && (cd rp4 && sha256sum -c SHA256SUMS)
   gh release download lab-images-rk3399 -R cz172638/zephyr -p '152-*'
   ```

### Jak workflow běží

```
lab-images.yml
  build (matice espressobin, rock_pi_4; fail-fast: false), runs-on ubuntu-24.04
    container ghcr.io/zephyrproject-rtos/ci:v0.29.4  (root)
    checkout → ws/zephyr (depth 1)
    cache ws/modules + ws/fw  (klíč: west.yml + skript desky + common.sh)
    bash -eu ws/zephyr/scripts/ci/lab/<deska>.sh   → out/
    upload-artifact <deska>-images (30 dní)
  release (release=true): všechny artefakty → prerelease lab-images-<větev>
```

### Na co si dát pozor

- **Velikost obrazu `ci`:** má jednotky GB komprimovaně, rozbalený přes 10 GB. Na hostovaném `ubuntu-24.04` se vejde, ale místa moc nezbývá. Pokud by docházelo:
  - Zúžit matici na jednu desku.
  - Místo `container:` udělat krok, který nejdřív uvolní disk (`rm -rf /usr/share/dotnet /opt/ghc /usr/local/lib/android`) a pak spustí `docker run … ghcr.io/zephyrproject-rtos/ci:v0.29.4 bash -eu …` jako v sekci 3.
- **Upstream workflowy** (twister, compliance) ve větvi `rk3399` se ve forku spustí při pushi, pokud nejsou vypnuté. Vypnutí přes `gh workflow disable` platí podle cesty souboru pro všechny větve. Postup je v `fork-ci-images.md`.
- **Vztah ke starší větvi `ci`** (`fork-ci-images.md`): tam běží `ubuntu-24.04` s vlastní instalací SDK (`sdk_install`) a skripty žijí v orphan větvi. Tady je SDK z obrazu `ci` a skripty jsou přímo ve větvi desky, takže jeden checkout dá zdrojáky i postup. Obojí může běžet vedle sebe.
- **Na železe** nic z CI ověřené není. Lokální běh skriptů z čistého workspace (sekce 5) ověřuje jen to, že se image sestaví.

## 5. Ověření

Oba skripty jsem pustil z čistého workspace na hostiteli s SDK 1.0.1: nový klon větve `rk3399`, `west update` s filtrem z GitHubu, firmware z lokálních repozitářů přes `file://`.

| Skript | Čas | Výstup |
|---|---|---|
| `rock_pi_4.sh` | 2 min 17 s | `152-rp4-storage-sd.bin` (48 MiB), `-spi.bin`, README, SHA256SUMS |
| `espressobin.sh` | 4 min 26 s | `150-lab-mods-{spi,sata}.bin`, `-uart.tgz`, 9× `150-*.llext` (kontrola symbolů `lab_mods` prošla), `lab.pl`, `sata-install.sh` |

Kontejner ani Actions jsem nespouštěl, protože tady není Docker. Rozdíl proti hostiteli je jen v balíčcích (skript je doinstaluje) a v cestě k SDK/venv.

ESPRESSObin z větve `rk3399` nese číslo 150 ze snippetu, ale obsahuje novější kód než 150 na deskách (`pcie_ctrl_is_rc()` v EPF a lab agentu). Pro nasazení ho postav s `IMG=153`, jinak se `.llext` načtou do jiného firmwaru, než pro který byly přeložené.
