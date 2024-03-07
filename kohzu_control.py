import kohzu_controller_annotated as kc


# This dictionary takes the motor as a string "motor number_coordinate_translation/rotation" and returns the right code number
motor_dict = {'g1_x_tra': 1, 'g1_y_tra': 2, 'g1_z_tra': 3, 
             'g1_x_rot': 4, 'g1_y_rot': 5, 'g1_z_rot': 6, 
             'g2_x_tra': 7, 'g2_y_tra': 8, 'g2_z_tra': 9, 
             'g2_x_rot': 10, 'g2_y_rot': 11, 'g2_z_rot': 12,
             'g3_x_tra': 13, 'g3_y_tra': 14, 'g3_z_tra': 15, 
             'g3_x_rot': 16, 'g3_y_rot': 17, 'g3_z_rot': 18}

# Each motor moves in 'steps', this dictionary converts 'steps; to units of millimeters (for translation) or degrees (for rotation)
motor_units_mm = {'g1_x_tra': 0.25e-3, 'g1_y_tra': 0.05e-3, 'g1_z_tra': 0.25e-3,
                  'g1_x_rot': 0.60e-3, 'g1_y_rot': 2.00e-3, 'g1_z_rot': 0.60e-3,
                  'g2_x_tra': 0.25e-3, 'g2_y_tra': 0.05e-3, 'g2_z_tra': 0.25e-3,
                  'g2_x_rot': 0.60e-3, 'g2_y_rot': 2.00e-3, 'g2_z_rot': 0.60e-3,
                  'g3_x_tra': 0.25e-3, 'g3_y_tra': 0.05e-3, 'g3_z_tra': 0.25e-3,
                  'g3_x_rot': 0.60e-3, 'g3_y_rot': 2.00e-3, 'g3_z_rot': 0.60e-3}

# This dictionary returns the full name of each motor, for convinence 
motor_name_strs = {'g1_x_tra': 'G1 X Translation', 'g1_y_tra': 'G1 Y Translation', 'g1_z_tra': 'G1 Z Translation',
                  'g1_x_rot': 'G1 X Rotation', 'g1_y_rot': 'G1 Y Rotation', 'g1_z_rot': 'G1 Z Rotation',
                  'g2_x_tra': 'G2 X Translation', 'g2_y_tra': 'G2 Y Translation', 'g2_z_tra': 'G2 Z Translation',
                  'g2_x_rot': 'G2 X Rotation', 'g2_y_rot': 'G2 Y Rotation', 'g2_z_rot': 'G2 Z Rotation',
                  'g3_x_tra': 'G3 X Translation', 'g3_y_tra': 'G3 Y Translation', 'g3_z_tra': 'G3 Z Translation',
                  'g3_x_rot': 'G3 X Rotation', 'g3_y_rot': 'G3 Y Rotation', 'g3_z_rot': 'G3 Z Rotation'}


kohzu = kc.Kohzu_Controller('com5')
kohzu.axis = tuple(motor_dict.keys())
kohzu.axis_numbers = motor_dict
kohzu.axis_names = motor_name_strs
kohzu.axis_units = motor_units_mm

