#!/usr/bin/env bash

IMAGE=rvizweb

rm -rf traj_manager
mkdir traj_manager
cp -r /home/wyatt/traj_manager/ .

pushd "$( dirname "${BASH_SOURCE[0]}" )"
docker build -t ${IMAGE} "$@" . $1
popd

