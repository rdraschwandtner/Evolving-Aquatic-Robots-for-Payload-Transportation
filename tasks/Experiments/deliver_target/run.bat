@echo off
python main.py --targetposx=40 --targetposy=40
move 0_logged_output.dat 1_0_logged_output.dat
move avg_bodypos.out 1avg_bodypos.out
move body_0_pos.out 1body_0_pos.out
move targetpos.out 1targetpos.out

python main.py --targetposx=-40 --targetposy=40
move 0_logged_output.dat 2_0_logged_output.dat
move avg_bodypos.out 2avg_bodypos.out
move body_0_pos.out 2body_0_pos.out
move targetpos.out 2targetpos.out

python main.py --targetposx=40 --targetposy=-40
move 0_logged_output.dat 3_0_logged_output.dat
move avg_bodypos.out 3avg_bodypos.out
move body_0_pos.out 3body_0_pos.out
move targetpos.out 3targetpos.out

python main.py --targetposx=-40 --targetposy=-40
move 0_logged_output.dat 4_0_logged_output.dat
move avg_bodypos.out 4avg_bodypos.out
move body_0_pos.out 4body_0_pos.out
move targetpos.out 4targetpos.out