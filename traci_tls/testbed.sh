python3 runner.py --saverewards 'fixed' --method 'f' --nogui
python3 runner.py --epsilon 0.1 --saverewards qlearn_delay --savetofile Qvalues_delay_stage1 --method qle --nogui
python3 runner.py --saverewards qgreedy_delay --loadfromfile Qvalues_delay_stage1_episode0 --method qg --nogui
python3 runner.py --epsilon 0.1 --method 'dql' --savetofile dtc_params
