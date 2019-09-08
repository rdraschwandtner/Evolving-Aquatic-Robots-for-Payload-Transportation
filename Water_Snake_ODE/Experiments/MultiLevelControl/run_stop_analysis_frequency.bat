@echo off
python multilevel_ctrl.py --log_frames --stop --eval_time=20 --rdensity=50 --freq=0.2
move 0_logged_output.dat f02_0_logged_output.dat
move avg_bodypos.out f02avg_bodypos.out
move body_0_pos.out f02body_0_pos.out
move body.out f02body.out

python multilevel_ctrl.py --log_frames --stop --eval_time=20 --rdensity=50 --freq=0.4
move 0_logged_output.dat f04_0_logged_output.dat
move avg_bodypos.out f04avg_bodypos.out
move body_0_pos.out f04body_0_pos.out
move body.out f04body.out

python multilevel_ctrl.py --log_frames --stop --eval_time=20 --rdensity=50 --freq=0.6
move 0_logged_output.dat f06_0_logged_output.dat
move avg_bodypos.out f06avg_bodypos.out
move body_0_pos.out f06body_0_pos.out
move body.out f06body.out

python multilevel_ctrl.py --log_frames --stop --eval_time=20 --rdensity=50 --freq=1.0
move 0_logged_output.dat f10_0_logged_output.dat
move avg_bodypos.out f10avg_bodypos.out
move body_0_pos.out f10body_0_pos.out
move body.out f10body.out

python multilevel_ctrl.py --log_frames --stop --eval_time=20 --rdensity=50 --freq=2.0
move 0_logged_output.dat f20_0_logged_output.dat
move avg_bodypos.out f20avg_bodypos.out
move body_0_pos.out f20body_0_pos.out
move body.out f20body.out

python multilevel_ctrl.py --log_frames --stop --eval_time=20 --rdensity=50 --freq=3.0
move 0_logged_output.dat f30_0_logged_output.dat
move avg_bodypos.out f30avg_bodypos.out
move body_0_pos.out f30body_0_pos.out
move body.out f30body.out

python multilevel_ctrl.py --log_frames --stop --eval_time=20 --rdensity=50 --freq=4.0
move 0_logged_output.dat f40_0_logged_output.dat
move avg_bodypos.out f40avg_bodypos.out
move body_0_pos.out f40body_0_pos.out
move body.out f40body.out