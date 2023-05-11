
# Building Protogen

## Orion SDK depends on ProtoGen
ProtoGen is an application that can take a commonly defined protocol definition from an 
XML file and generate the proper .h/.c files for a given architecture.  We include 
compiled versions for a number of platforms, but if needed it can be compiled for your
given platform.

## Protogen details for use with Orion SDK
* Source Location: https://github.com/billvaglienti/ProtoGen
* Version: 2.12.d
* git revision:  a0472b7ca056c14e5c036805bfa7548623c865ce

## Building Protogen
* Refer to the Protogen README.md for instructions and dependencies

```bash
git clone https://github.com/billvaglienti/ProtoGen.git
cd ProtoGen
git checkout a0472b7ca056c14e5c036805bfa7548623c865ce
qmake && make
``` 
