p_m = [0.5 2 1]'
p_b_exp = [p_m(1), -p_m(3), p_m(2)]'

rotm = [1 0 0 ; 0 0 -1 ; 0 1 0]

p_b = rotm * p_m