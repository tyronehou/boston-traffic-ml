#!/bin/bash
mkdir results/single/dql_rho
for rho in 1 1.5 2 2.5 3
do
results_dir=results/single/dql_rho/dql_rho=$rho
mkdir $results_dir
python3 multirunner.py --intersection 's' --epsilon 0.1 --rho $rho --method 'dql' --savetofile dql_params --episodes 30 --nogui
mv dql_params* $results_dir
mv dql_episode* $results_dir
mv simulation_params $results_dir
done
