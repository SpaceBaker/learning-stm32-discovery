# Make Project Template

The purpose of this template is to be a starter point for new projects that intend to use 'GNU Make' for build automation, 'clangd' for adding smart features to the text editor (namely VSCode), and cppcheck for static C/C++ code analysis.

## Getting Started

### Dependencies

Assumes that you are working in a linux developement environement (Native, VM, WSL or Container)
* GNU Make 4.3 or later
* clangd 17.0.3 or later
* cppcheck 2.13.0 or later
    * GCC (g++) 5.1 or later
    * cmake 3.5 or later
* compiledb
    * python 2.7 or 3.6
    * pip 22.0.2 or later

### Installing

Feel free to install the dependencies the way you prefer, this is just the way I did it.

#### GNU Make

Install through apt :

```
sudo apt install make
```

#### clangd

I'm using a newer version than the one that is available through apt.
Visit clangd [website](https://clangd.llvm.org/installation) for the latest version.


```
wget https://github.com/clangd/clangd/releases/download/17.0.3/clangd-linux-17.0.3.zip
unzip clangd-linux-17.0.3.zip
```

clangd is now where you unziped it.

#### cppcheck

1. If not already installed, install g++ and cmake

```
sudo apt install g++ cmake
```

I'm using a newer version than the one that is available through apt.
Visit cppcheck [website](https://github.com/danmar/cppcheck) for it's latest version.

2. Download the source code or clone the github repo

```
wget https://github.com/danmar/cppcheck/archive/2.13.0.tar.gz
tar -xzf 2.13.0.tar.gz
```

3. Compile using cmake

```
mkdir build
cd build
cmake ..
cmake --build .
```

cppcheck is now installed in the build/bin directory

#### compiledb

1. If not already installed, install python and pip

```
sudo apt install pyhton3 pyhton3-pip
```

2. Install compiledb

```
pip install compiledb
```

## Troubleshooting

TODO

## Authors

[@SpaceBaker](https://github.com/SpaceBaker)

## Version History

* TODO

## License

This project is licensed under the MIT License - see the LICENSE file for details
