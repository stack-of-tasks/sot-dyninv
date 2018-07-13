sot-dyninv
==========

[![Building Status](https://travis-ci.org/stack-of-tasks/sot-dyninv.svg?branch=master)](https://travis-ci.org/stack-of-tasks/sot-dyninv)
[![Pipeline status](https://gepgitlab.laas.fr/stack-of-tasks/sot-dyninv/badges/master/pipeline.svg)](https://gepgitlab.laas.fr/stack-of-tasks/sot-dyninv/commits/master)
[![Coverage report](https://gepgitlab.laas.fr/stack-of-tasks/sot-dyninv/badges/master/coverage.svg?job=doc-coverage)](http://projects.laas.fr/stack-of-tasks/doc/stack-of-tasks/sot-dyninv/master/coverage/)


This sofware provides the solver and features to resolve the inverse of the
dynamics of a free-floating robot, in contact with its environment.  The basics
are provided by the sot core, and the computation of the dynamics of the robot
can be computed by a third party lib, for example sot-dynamic.


Setup
-----

To compile this package, it is recommended to create a separate build
directory:

    mkdir _build
    cd _build
    cmake [OPTIONS] ..
    make install

Please note that CMake produces a `CMakeCache.txt` file which should
be deleted to reconfigure a package from scratch.


### Dependencies

The sot dynamic inverse depends on several packages which
have to be available on your machine.

 - Libraries:
   - sot-core (>= 1.0.0)
   - soth (>= 0.0.1)
 - System tools:
   - CMake (>=2.6)
   - pkg-config
   - usual compilation tools (GCC/G++, make, etc.)


[sot-core]: http://github.com/stack-of-tasks/sot-core
[soth]: https://github.com/stack-of-tasks/soth.git
