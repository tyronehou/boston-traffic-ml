python3 multirunner.py --saverewards 'fixed' --method 'f' --nogui
python3 multirunner.py --epsilon 0.1 --saverewards qlearn_delay --savetofile Qvalues_delay_stage1 --method qle --nogui
python3 multirunner.py --epsilon 0.1 --method 'dql' --savetofile dtc_params --intersection 'm' --episodes 30 --nogui
python3 multirunner.py --epsilon 0.1 --method 'marl' --savetofile marl_params --intersection 'm' --episodes 30 --nogui
