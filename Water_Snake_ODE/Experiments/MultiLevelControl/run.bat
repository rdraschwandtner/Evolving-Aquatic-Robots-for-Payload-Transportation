@echo off
python multilevel_ctrl.py --log_frames --targetposx=10 --targetposy=10
move 0_logged_output.dat 1_0_logged_output.dat
move avg_bodypos.out 1avg_bodypos.out
move body_0_pos.out 1body_0_pos.out

python multilevel_ctrl.py --log_frames --targetposx=-10 --targetposy=10
move 0_logged_output.dat 2_0_logged_output.dat
move avg_bodypos.out 2avg_bodypos.out
move body_0_pos.out 2body_0_pos.out

python multilevel_ctrl.py --log_frames --targetposx=10 --targetposy=-10
move 0_logged_output.dat 3_0_logged_output.dat
move avg_bodypos.out 3avg_bodypos.out
move body_0_pos.out 3body_0_pos.out

python multilevel_ctrl.py --log_frames --targetposx=-10 --targetposy=-10
move 0_logged_output.dat 4_0_logged_output.dat
move avg_bodypos.out 4avg_bodypos.out
move body_0_pos.out 4body_0_pos.out