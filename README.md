# README #

This README would normally document whatever steps are necessary to get your application up and running.

### What is this repository for? ###

* Quick summary
* Version
* [Learn Markdown](https://bitbucket.org/tutorials/markdowndemo)

### How do I get set up? ###

* Summary of set up
* Configuration
* Dependencies
* Database configuration
* How to run tests
* Deployment instructions

### Contribution guidelines ###

* Writing tests
* Code review
* Other guidelines

### Who do I talk to? ###

* Repo owner or admin
* Other community or team contact

### Dependancy 
* libboost BGL 1.58
* libxerces 3.1


### Building Kheops ###
* cmake .
* make

### Help menu ###
* ./kheops -h 

### Run Kheops ###
* ./kheops -s xml_path 

### User function exampe ###

* Building demofct by hand :
mkdir lib
g++ -c user_src/demofct.cpp -o user_src/demofct.o -I include -I user_src -I /usr/include/eigen3 -fpic -std=c++17
gcc -shared -o  lib/demofct.so  user_src/demofct.o

