@echo off
python multilevel_ctrl.py --log_frames --stop --eval_time=20 --rdensity=50
move 0_logged_output.dat d50_0_logged_output.dat
move avg_bodypos.out d50avg_bodypos.out
move body_0_pos.out d50body_0_pos.out
move body.out d50body.out

python multilevel_ctrl.py --log_frames --stop --eval_time=20 --rdensity=8
move 0_logged_output.dat d8_0_logged_output.dat
move avg_bodypos.out d8avg_bodypos.out
move body_0_pos.out d8body_0_pos.out
move body.out d8body.out

python multilevel_ctrl.py --log_frames --stop --eval_time=20 --rdensity=12
move 0_logged_output.dat d12_0_logged_output.dat
move avg_bodypos.out d12avg_bodypos.out
move body_0_pos.out d12body_0_pos.out
move body.out d12body.out

python multilevel_ctrl.py --log_frames --stop --eval_time=20 --rdensity=25
move 0_logged_output.dat d25_0_logged_output.dat
move avg_bodypos.out d25avg_bodypos.out
move body_0_pos.out d25body_0_pos.out
move body.out d25body.out

python multilevel_ctrl.py --log_frames --stop --eval_time=20 --rdensity=200
move 0_logged_output.dat d200_0_logged_output.dat
move avg_bodypos.out d200avg_bodypos.out
move body_0_pos.out d200body_0_pos.out
move body.out d200body.out

python multilevel_ctrl.py --log_frames --stop --eval_time=20 --rdensity=500
move 0_logged_output.dat d500_0_logged_output.dat
move avg_bodypos.out d500avg_bodypos.out
move body_0_pos.out d500body_0_pos.out
move body.out d500body.out