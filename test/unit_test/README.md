# Install GTest Framework

1. Get the framework from GitHub 

` wget https://github.com/google/googletest/releases/download/v1.17.0/googletest-1.17.0.tar.gz ` release at the time of writing, choose whatever most recent release

2. Unpack and build

``` 
tar xf googletest-1.17.0.tar.gz
cd googletest-1.17.0/
cmake -DBUILD_SHARED_LIBS=ON .
make
```

3. Install

`sudo make install`
