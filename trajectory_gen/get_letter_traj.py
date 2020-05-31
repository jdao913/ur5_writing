import numpy as np
import csv

def get_x_y(csv_file):
    x = []
    y = []
    with open(csv_file) as csvfile:
        plots = csv.reader(csvfile,delimiter=',')
        for row in plots:
            x.append(float(row[0]))
            y.append(float(row[1]))

    return x,y

def get_traj(letter):
    # alpha = numer in alphabet
    # beta = 0 for lowercase
    # beta = 1 for uppercase

    if letter == 'A':
        xd,yd = get_x_y('capital_letters_csvs/a_cap.csv')
        len_traj = len(xd)
    elif letter == 'B':
        xd,yd = get_x_y('capital_letters_csvs/b_cap.csv')
        len_traj = len(xd)
    elif letter == 'C':
        xd,yd = get_x_y('capital_letters_csvs/c_cap.csv')
        len_traj = len(xd)
    elif letter == 'D':
        xd,yd = get_x_y('capital_letters_csvs/d_cap.csv')
        len_traj = len(xd)
    elif letter == 'E':
        xd,yd = get_x_y('capital_letters_csvs/e_cap.csv')
        len_traj = len(xd)
    elif letter == 'F':
        xd,yd = get_x_y('capital_letters_csvs/e_cap.csv')
        len_traj = len(xd)
    elif letter == 'G':
        xd,yd = get_x_y('capital_letters_csvs/e_cap.csv')
        len_traj = len(xd)
    elif letter == 'H':
        xd,yd = get_x_y('capital_letters_csvs/h_cap.csv')
        len_traj = len(xd)
    elif letter == 'I':
        xd,yd = get_x_y('capital_letters_csvs/i_cap.csv')
        len_traj = len(xd)
    elif letter == 'J':
        xd,yd = get_x_y('capital_letters_csvs/j_cap.csv')
        len_traj = len(xd)
    elif letter == 'K':
        xd,yd = get_x_y('capital_letters_csvs/k_cap.csv')
        len_traj = len(xd)
    elif letter == 'L':
        xd,yd = get_x_y('capital_letters_csvs/l_cap.csv')
        len_traj = len(xd)
    elif letter == 'M':
        xd,yd = get_x_y('capital_letters_csvs/m_cap.csv')
        len_traj = len(xd)
    elif letter == 'N':
        xd,yd = get_x_y('capital_letters_csvs/n_cap.csv')
        len_traj = len(xd)
    elif letter == 'O':
        xd,yd = get_x_y('capital_letters_csvs/o_cap.csv')
        len_traj = len(xd)
    elif letter == 'P':
        xd,yd = get_x_y('capital_letters_csvs/p_cap.csv')
        len_traj = len(xd)
    elif letter == 'Q':
        xd,yd = get_x_y('capital_letters_csvs/q_cap.csv')
        len_traj = len(xd)
    elif letter == 'R':
        xd,yd = get_x_y('capital_letters_csvs/r_cap.csv')
        len_traj = len(xd)
    elif letter == 'S':
        xd,yd = get_x_y('capital_letters_csvs/s_cap.csv')
        len_traj = len(xd)
    elif letter == 'T':
        xd,yd = get_x_y('capital_letters_csvs/t_cap.csv')
        len_traj = len(xd)
    elif letter == 'U':
        xd,yd = get_x_y('capital_letters_csvs/u_cap.csv')
        len_traj = len(xd)
    elif letter == 'V':
        xd,yd = get_x_y('capital_letters_csvs/v_cap.csv')
        len_traj = len(xd)
    elif letter == 'W':
        xd,yd = get_x_y('capital_letters_csvs/w_cap.csv')
        len_traj = len(xd)
    elif letter == 'X':
        xd,yd = get_x_y('capital_letters_csvs/x_cap.csv')
        len_traj = len(xd)
    elif letter == 'Y':
        xd,yd = get_x_y('capital_letters_csvs/y_cap.csv')
        len_traj = len(xd)
    elif letter == 'Z':
        xd,yd = get_x_y('capital_letters_csvs/z_cap.csv')
        len_traj = len(xd)
    elif letter == 'a':
        xd,yd = get_x_y('small_letters_csvs/a_small.csv')
        len_traj = len(xd)
    elif letter == 'b':
        xd,yd = get_x_y('small_letters_csvs/b_small.csv')
        len_traj = len(xd)
    elif letter == 'c':
        xd,yd = get_x_y('small_letters_csvs/c_small.csv')
        len_traj = len(xd)
    elif letter == 'd':
        xd,yd = get_x_y('small_letters_csvs/d_small.csv')
        len_traj = len(xd)
    elif letter == 'e':
        xd,yd = get_x_y('small_letters_csvs/e_small.csv')
        len_traj = len(xd)
    elif letter == 'f':
        xd,yd = get_x_y('small_letters_csvs/f_small.csv')
        len_traj = len(xd)
    elif letter == 'g':
        xd,yd = get_x_y('small_letters_csvs/g_small.csv')
        len_traj = len(xd)
    elif letter == 'h':
        xd,yd = get_x_y('small_letters_csvs/h_small.csv')
        len_traj = len(xd)
    elif letter == 'i':
        xd,yd = get_x_y('small_letters_csvs/i_small.csv')
        len_traj = len(xd)
    elif letter == 'j':
        xd,yd = get_x_y('small_letters_csvs/j_small.csv')
        len_traj = len(xd)
    elif letter == 'k':
        xd,yd = get_x_y('small_letters_csvs/k_small.csv')
        len_traj = len(xd)
    elif letter == 'l':
        xd,yd = get_x_y('small_letters_csvs/l_small.csv')
        len_traj = len(xd)
    elif letter == 'm':
        xd,yd = get_x_y('small_letters_csvs/m_small.csv')
        len_traj = len(xd)
    elif letter == 'n':
        xd,yd = get_x_y('small_letters_csvs/n_small.csv')
        len_traj = len(xd)
    elif letter == 'o':
        xd,yd = get_x_y('small_letters_csvs/o_small.csv')
        len_traj = len(xd)
    elif letter == 'p':
        xd,yd = get_x_y('small_letters_csvs/p_small.csv')
        len_traj = len(xd)
    elif letter == 'q':
        xd,yd = get_x_y('small_letters_csvs/q_small.csv')
        len_traj = len(xd)
    elif letter == 'r':
        xd,yd = get_x_y('small_letters_csvs/r_small.csv')
        len_traj = len(xd)
    elif letter == 's':
        xd,yd = get_x_y('small_letters_csvs/s_small.csv')
        len_traj = len(xd)
    elif letter == 't':
        xd,yd = get_x_y('small_letters_csvs/t_small.csv')
        len_traj = len(xd)
    elif letter == 'u':
        xd,yd = get_x_y('small_letters_csvs/u_small.csv')
        len_traj = len(xd)
    elif letter == 'v':
        xd,yd = get_x_y('small_letters_csvs/v_small.csv')
        len_traj = len(xd)
    elif letter == 'w':
        xd,yd = get_x_y('small_letters_csvs/w_small.csv')
        len_traj = len(xd)
    elif letter == 'x':
        xd,yd = get_x_y('small_letters_csvs/x_small.csv')
        len_traj = len(xd)
    elif letter == 'y':
        xd,yd = get_x_y('small_letters_csvs/y_small.csv')
        len_traj = len(xd)
    elif letter == 'z':
        xd,yd = get_x_y('small_letters_csvs/z_small.csv')
        len_traj = len(xd)
    else:
        xd = []
        yd = []
        l = 0
        print("Fatal Error:")
        print("Help this is not a letter")


    #traj = np.array([beta,alpha],[beta,alpha+1],[beta,alpha+2])
    return xd,yd,len_traj
    #return xd,yd,xdotd,ydotd,xddotd,yddotd
