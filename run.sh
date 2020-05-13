#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
DATA="$DIR/data/"
g++ src/gen_data/gen_1.cpp -o bin/gen_data
bin/gen_data "$DATA"
cd "$DIR"/build
cmake "$DIR"/src
make
cd "$DIR"
bin/sim "$DATA"
bin/viewer -o "$DATA"/main -s src/shaders
