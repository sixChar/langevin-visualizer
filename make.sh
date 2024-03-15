mkdir build
pushd build
gcc ../src/main.c -o main -g -Wall -lSDL2 -lm
popd
