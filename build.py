#!/bin/python3
import os
import glob
import sys
import subprocess
import threading
from script.util import parseSize, compareFiles
import atexit

def readConfig(path: str) -> dict[str, list[str]]:
    """Reads a configuration file and returns a dictionary of lists of strings."""
    config = {}
    try:
        with open(path, "r") as f:
            lines = f.readlines()
            for line in lines:
                line = line.strip()
                if ':' in line:
                    key, value = line.split(":", 1)
                    key = key.strip()
                    value = value.strip().strip("'")
                    config[key] = [v.strip().strip("'") for v in value.split(",")]
    except FileNotFoundError:
        print(f"Error: Configuration file '{path}' not found.")
        exit(1)
    return config


def writeConfig(config: dict[str, list[str]], path: str) -> None:
    """Writes a configuration dictionary to a file."""
    with open(path, "w") as f:
        for key, values in config.items():
            f.write(f"{key}:")
            for val in enumerate(values):
                f.write(f"'{val[1]}'")
                if val[0]+1 < len(values):
                    f.write(", ")
            f.write('\n')

def checkConfig(config: dict[str, list[str]], allowed_config: list[list[str, list[str], bool]]) -> bool:
    for (key, valid_values, is_mandatory) in allowed_config:
        if key not in config:
            if is_mandatory:
                print(f"Mandatory config {key} is missing")
                return False
            continue
        if valid_values:
            for value in config[key]:
                if value not in valid_values:
                    print(f"Invalid value '{value}' for key '{key}'")
                    return False
    return True

CONFIG = readConfig("./script/config.py")
OLD_CONFIG = {}
if os.path.exists("./script/config.py.old"):
    OLD_CONFIG = readConfig("./script/config.py.old")
ALLOWED_CONFIG = [
    ["config", ["release", "debug"], True],
    ["compiler", ["gcc", "clang"], True],
    ["outDir", [], True],
    ["analyzer", ["yes", "no"], True],
    ["usan", ["yes", "no"], True],
    ["asan", ["yes", "no"], True],
]
if not checkConfig(CONFIG, ALLOWED_CONFIG):
    print("Invalid config file.")
    print("Allowed config items")
    for option in ALLOWED_CONFIG:
        name = option[0]
        values = option[1]
        required = option[2]
        print(f"{name} (required = {required})")
        if len(values) == 0:
            print("    This can be anything as long as it's provided")
        else:
            for val in values:
                print(f"  - {val}")
    exit(1)
writeConfig(CONFIG, "./script/config.py.old")
force_rebuild = False
if OLD_CONFIG != CONFIG:
    force_rebuild = True
    print("Configuration changed, rebuilding...")
CONFIG["CFLAGS"] = ['-c', '-DCOMPILE']
CONFIG["CFLAGS"] += ['-finline-functions']
CONFIG["CFLAGS"] += ['-Werror', '-Wall', '-Wextra', '-Wpointer-arith', '-Wshadow']
CONFIG["CFLAGS"] += ['-march=native', '-mtune=native']
CONFIG["CXXFLAGS"] = []
CONFIG["ASFLAGS"] = ['-felf64']
CONFIG["LDFLAGS"] = ['-Wl,--build-id=none', '-ffunction-sections', '-fdata-sections', '-Oz']
CONFIG["INCPATHS"] = ['-Iinclude', '-I /usr/include', '-I./']
if "imageSize" not in CONFIG:
    CONFIG["imageSize"] = '128m'

if "yes" in CONFIG.get("lto"):
    CONFIG["CFLAGS"] += ['-flto']
    CONFIG["LDFLAGS"] += ['-flto=auto']
else:
    CONFIG["CFLAGS"] += ['-fno-lto']
    CONFIG["LDFLAGS"] += ['-fno-lto']

if "debug" in CONFIG.get("config"):
    CONFIG["CFLAGS"] += ["-O0"]
    CONFIG["CFLAGS"] += ["-DDEBUG"]
else:
    CONFIG["CFLAGS"] += ["-O2"]
    CONFIG["CFLAGS"] += ["-DNDEBUG"]

if "yes" in CONFIG.get("usan"):
    CONFIG["CFLAGS"] += ['-fsanitize=undefined']
    if "clang" not in CONFIG.get("compiler"):
        CONFIG["LDFLAGS"] += ['-fsanitize=undefined']
        
if "yes" in CONFIG.get("asan"):
    CONFIG["CFLAGS"] += ['-fsanitize=address']
    if "clang" not in CONFIG.get("compiler"):
        CONFIG["LDFLAGS"] += ['-fsanitize=address']

if "gcc" in CONFIG.get("compiler"):
    if "yes" in CONFIG.get("analyzer"):
        CONFIG["CFLAGS"].append("-fanalyzer")
    CONFIG["CFLAGS"] += ['-finline-functions-called-once', '-fpeel-loops', '-fprefetch-loop-arrays', '-fmax-errors=1']
    CONFIG["CFLAGS"] += ['-Wno-aggressive-loop-optimizations']
if "clang" in CONFIG.get("compiler"):
    CONFIG["CFLAGS"] += ['-Wno-ignored-attributes']

stopEvent = threading.Event()

def callCmd(command, print_out=False):
    with open("commands.txt", "a") as f:
        f.write(command+'\n')
    result = subprocess.run(command, capture_output=not print_out, text=True, shell=True)
    if result.returncode != 0:
        print(result.stderr)
    return [result.returncode, result.stdout]

callCmd("rm -rf commands.txt")

if not compareFiles('build.py', '.build-cache/build.py'):
    if not os.path.exists(".build-cache"):
        callCmd(f"mkdir -p .build-cache")
    callCmd(f"cp build.py ./.build-cache/build.py")
    force_rebuild = True


def checkExtension(file: str, valid_extensions: list[str]):
    for ext in valid_extensions:
        if file.endswith(ext):
            return True
    return False

def getExtension(file):
    return file.split(".")[-1]

def buildC(file):
    compiler = CONFIG.get("compiler")[0]
    # if CONFIG.get("compiler")[0] == "gcc":
    #     compiler += "-11"
    options = CONFIG["CFLAGS"].copy()
    options.append("-std=c11")
    command = compiler + " " + file
    for option in options:
        command += " " + option
    print(f"C     {file}")
    command += f" -o {CONFIG['outDir'][0]}/{file}.o"
    return callCmd(command, True)[0]
    
def buildCXX(file):
    compiler = CONFIG.get("compiler")[0]
    if compiler == "gcc":
        compiler = "g"
    compiler += "++"
    # if CONFIG.get("compiler")[0] == "gcc":
    #     compiler += "-11"
    options = CONFIG["CFLAGS"].copy()
    options += CONFIG["CXXFLAGS"].copy()
    options.append("-std=c++26")
    if "yes" in CONFIG.get("analyzer"):
        options.remove("-fanalyzer")
    command = compiler + " " + file
    for option in options:
        command += " " + option
    print(f"CXX   {file}")
    command += f" -o {CONFIG['outDir'][0]}/{file}.o"
    return callCmd(command, True)[0]

def buildASM(file):
    compiler = "nasm"
    options = CONFIG["ASFLAGS"].copy()
    command = compiler + " " + file
    for option in options:
        command += " " + option
    print(f"AS    {file}")
    command += f" -o {CONFIG['outDir'][0]}/{file}.o"
    return callCmd(command, True)[0]

def buildAR(dir: str, out_file: str):
    files = glob.glob(f"{dir}/**", recursive=True)
    obj_files = []
    for file in files:
        if not os.path.isfile(file):
            continue
        if not checkExtension(file, ["o", "bc"]):
            continue
        obj_files.append(file)
    obj_files_str = " ".join(obj_files)
    cmd = f"ar rcs {out_file} {obj_files_str}"
    print(f"AR    {out_file}")
    callCmd(cmd)

def buildKernel(kernel_dir: str, basename: str):
    files = glob.glob(kernel_dir+'/**', recursive=True)
    files = sorted(files, key=str.lower)
    CONFIG["INCPATHS"] += [f"-I{kernel_dir}"]
    for file in files:
        if not os.path.isfile(file):
            continue
        if not checkExtension(file, ["c", "cc", "cpp", "asm"]):
            continue
        if getExtension(file) == "inc" or getExtension(file) == "h":
            continue
        str_paths = ""
        for incPath in CONFIG["INCPATHS"]:
            str_paths += f" {incPath}"
        code, _ = callCmd(f"cpp {str_paths} -D_GLIBCXX_HOSTED=1 {file} -o ./tmp.txt", True)
        if code != 0:
            print(f"CPP failed to pre process {file}")
            exit(code)
        if not force_rebuild and compareFiles("./tmp.txt", os.path.abspath(f"./.build-cache/{basename}/cache/{file}")):
            continue
        callCmd(f"mkdir -p {CONFIG['outDir'][0]}/{os.path.dirname(file)}")
        callCmd(f"mkdir -p ./.build-cache/{basename}/cache/{os.path.dirname(file)}")
        callCmd(f"cp ./tmp.txt ./.build-cache/{basename}/cache/{file}")
        code = 0
        CONFIG["CFLAGS"] += CONFIG["INCPATHS"]
        CONFIG["ASFLAGS"] += CONFIG["INCPATHS"]
        if getExtension(file) == "cc" or getExtension(file) == "cpp" or getExtension(file) == "c":
            print(f"FMT   {file}")
            callCmd(f"clang-format -i {file}")
        if getExtension(file) == "c":
            code = buildC(file)
        elif getExtension(file) == "asm":
            code = buildASM(file)
        elif getExtension(file) == "cc" or getExtension(file) == "cpp":
            code = buildCXX(file)
        else:
            print(f"Invalid or unhandled extension `{getExtension(file)}` on file {file}")
            exit(1)

        for incPath in CONFIG["INCPATHS"]:
            CONFIG["CFLAGS"].remove(incPath)
            CONFIG["ASFLAGS"].remove(incPath)

        if code != 0:
            callCmd(f"rm -f ./.build-cache/{basename}/cache/{file}")
            exit(code)

def linkDir(kernel_dir, linker_file, static_lib_files=[], out_name="a"):
    files = glob.glob(kernel_dir+'/**', recursive=True)
    if "gcc" in CONFIG["compiler"]:
        command = "g++"
    else:
        command = "clang++ -fuse-ld=lld"
    options = CONFIG["LDFLAGS"]
    for option in options:
        command += " " + option
    for file in files:
        if not os.path.isfile(file):
            continue
        if not checkExtension(file, ["o", "bc"]):
            continue
        command += " " + file
    if linker_file != None:
        command += f" -Wl,-T {linker_file}"
        command +=  " -Wl,--no-whole-archive"
        command +=  " -Wl,--whole-archive"
    for static_lib in static_lib_files:
        command += f" {static_lib}"
    command += f" -Wl,-Map={CONFIG['outDir'][0]}/{out_name}.map"
    command += f" -o {CONFIG['outDir'][0]}/{out_name}.elf"
    file = f"{CONFIG['outDir'][0]}/{out_name}.elf"
    print(f"LD   {file}")
    if callCmd(command, True)[0] != 0:
        print(f"LD   {file} Failed")
        exit(1)
    callCmd(f"objdump -C -D -x -Mintel -g -r -t -L {CONFIG['outDir'][0]}/{out_name}.elf > {CONFIG['outDir'][0]}/{out_name}.asm")

def makeImageFile(out_file):
    size = parseSize(CONFIG["imageSize"][0])
    divSize = parseSize("1M")
    command = f"dd if=/dev/zero of={out_file} bs=1M count={size//divSize}"
    print("> Making image file")
    callCmd(command)

def makePartitionTable(out_file):
    print("> Making GPT partition")
    command = f"parted {out_file} --script mklabel gpt"
    callCmd(command)
    print("> Making EFI partition")
    command = f"parted {out_file} --script mkpart EFI FAT32 2048s 100MB"
    callCmd(command)
    print("> Making HOME partition")
    command = f"parted {out_file} --script mkpart HOME FAT32 100MB 100%"
    callCmd(command)
    print("> Setting EFI partition to be bootable")
    command = f"parted {out_file} --script set 1 boot on"
    callCmd(command)

def setupLoopDevice(out_file):
    print("> Setting up loop device")
    command = f"sudo losetup --show -f -P {out_file} > ./.build-cache/tmp.txt"
    callCmd(command)
    loop_device = ""
    with open("./.build-cache/tmp.txt") as f:
        loop_device = f.readline()
    loop_device = loop_device.strip()
    print(f"> Loop device: {loop_device}")
    return loop_device

def makeFileSystem(loop_device):
    print("> Formatting file systems")
    callCmd(f"sudo mkfs.fat -F32 {loop_device}p1")
    callCmd(f"sudo ./mkfs.fs {loop_device}p2")

def mountFs(device, boot, kernel, files):
    callCmd(f"mkdir -p mnt")
    print("> Copying boot files")
    callCmd(f"sudo mount {device}p1 mnt")
    callCmd(f"sudo mkdir -p mnt/EFI/BOOT")
    callCmd(f"sudo cp {boot} mnt/EFI/BOOT")
    callCmd(f"sudo cp {kernel} mnt")
    # callCmd(f"sudo cp ../init/bin/init mnt")
    callCmd(f"sudo echo \"Hello, World\" > mnt/test.txt")
    for file in files:
        if os.path.isfile(file):
            callCmd(f"sudo cp {file} mnt")
        else:
            callCmd(f"sudo mkdir mnt/{file}")
    if "limine-uefi" in CONFIG["bootloader"]:
        callCmd(f"sudo cp {CONFIG['outDir'][0]}/limine.conf mnt")
        callCmd(f"sudo cp {CONFIG['outDir'][0]}/limine-bios.sys mnt")
    callCmd(f"sudo umount -l mnt")
    callCmd(f"sudo losetup -d {device}")
    callCmd(f"rm -rf mnt")


def buildImage(out_file, boot_file, kernel_file, files):
    if not out_file.startswith("/dev"):
        callCmd(f"rm -f {out_file}")
        makeImageFile(out_file)
    makePartitionTable(out_file)
    LOOP_DEVICE=setupLoopDevice(out_file)
    makeFileSystem(LOOP_DEVICE)
    mountFs(LOOP_DEVICE, boot_file, kernel_file, files)
    if "limine-uefi" in CONFIG["bootloader"]:
        callCmd(f"./limine/bin/limine bios-install --no-gpt-to-mbr-isohybrid-conversion {out_file}")

def buildStaticLib(directory, out_file):
    os.makedirs(CONFIG["outDir"][0]+'/'+directory, exist_ok=True)
    CONFIG["INCPATHS"] += [f'-I{directory}']
    files = glob.glob(directory+'/**', recursive=True)
    files = sorted(files, key=str.lower)
    for file in files:
        if not os.path.isfile(file):
            continue
        if not checkExtension(file, ["c", "cc", "cpp", "asm"]):
            continue
        basename = os.path.basename(os.path.dirname(os.path.realpath(__file__)))
        str_paths = ""
        for incPath in CONFIG["INCPATHS"]:
            str_paths += f" {incPath}"
        code, _ = callCmd(f"cpp {str_paths} -D_GLIBCXX_HOSTED=1 {file} -o ./tmp.txt", True)
        if code != 0:
            print(f"CPP failed to pre process {file}")
            exit(code)
        if not force_rebuild and compareFiles("./tmp.txt", os.path.abspath(f"./.build-cache/{basename}/cache/{file}")):
            continue
        callCmd(f"mkdir -p {CONFIG['outDir'][0]}/{os.path.dirname(file)}")
        callCmd(f"mkdir -p ./.build-cache/{basename}/cache/{os.path.dirname(file)}")
        callCmd(f"cp ./tmp.txt ./.build-cache/{basename}/cache/{file}")
        code = 0
        CONFIG["CFLAGS"] += CONFIG["INCPATHS"]
        CONFIG["ASFLAGS"] += CONFIG["INCPATHS"]
        if getExtension(file) == "cc" or getExtension(file) == "cpp" or getExtension(file) == "c":
            print(f"FMT   {file}")
            callCmd(f"clang-format -i {file}")
        if getExtension(file) == "c":
            code = buildC(file)
        elif getExtension(file) == "asm":
            code = buildASM(file)
        elif getExtension(file) == "cc" or getExtension(file) == "cpp":
            code = buildCXX(file)
        else:
            print(f"Invalid or unhandled extension {getExtension(file)}")
            exit(1)

        for incPath in CONFIG["INCPATHS"]:
            CONFIG["CFLAGS"].remove(incPath)
            CONFIG["ASFLAGS"].remove(incPath)

        if code != 0:
            callCmd(f"rm -f ./.build-cache/{basename}/cache/{file}")
            exit(code)

    buildAR(f"{CONFIG["outDir"][0]}/{directory}", out_file)

def buildDir(directory, basename: str, static_lib: bool, out_file="a.out"):
    if static_lib:
        buildStaticLib(directory, out_file)
    else:
        buildKernel(directory, basename)

def getInfo():
    callCmd("rm -f info.txt")
    callCmd("touch info.txt")
    callCmd(f"cloc . --exclude-dir=limine,bin,.build-cache,script,.vscode >> info.txt")
    callCmd(f"tree -I 'bin' -I 'limine' -I 'script' -I '.vscode' -I 'tmp.txt' -I 'commands.txt' -I 'info.txt' >> info.txt")

def cleanFiles(dirs: list[str]):
    for dir_ in dirs:
        files = glob.glob(dir_+'/**', recursive=True)
        objFiles = glob.glob(CONFIG['outDir'][0]+'/'+dir_+'/**', recursive=True)
        newObjFiles = []
        for objFile in objFiles:
            if os.path.isfile(objFile) and checkExtension(objFile, ["o", ".bc"]):
                objFile.removesuffix(getExtension(objFile))
                newObjFiles.append(objFile)
        for file in files:
            if not os.path.isfile(file) or not checkExtension(file, ["c", "cc", "asm"]):
                continue
            file = CONFIG["outDir"][0] + '/' + file + '.o'
            if file not in newObjFiles:
                print(f"RM    {file}")
                callCmd(f"rm -f {file}")

def atExitFunc():
    currentUser = os.getlogin()
    callCmd(f"chown -R {currentUser}:{currentUser} ./")

dirs = ["assembler", "emulator"]

def main():
    # global force_rebuild
    atexit.register(atExitFunc)
    # if "clean" in sys.argv:
    #     force_rebuild = True
    if force_rebuild:
        print("Rebuilding...")
    if "compile" in sys.argv:
        for dir in dirs:
            print(f"> Building {dir}")
            os.chdir(dir)
            basename = dir
            if force_rebuild:
                callCmd(f"rm -rf ./.build-cache/{basename}")
                callCmd(f"rm -rf {CONFIG['outDir'][0]}")
            print("> Creating necesarry dirs")
            callCmd(f"mkdir -p {CONFIG['outDir'][0]}")
            callCmd(f"mkdir -p {CONFIG['outDir'][0]}/src")
            buildDir("src", dir, False)
            print("> Removing unused objects")
            cleanFiles(["src"])
            print(f"> Linking {dir}")
            linkDir(f"{CONFIG['outDir'][0]}/src", None, [], f"{dir}")
            print("> Getting info")
            getInfo()
            os.chdir("..")

if __name__ == '__main__':
    main()