function [T_trans, T_rot, T_thrust] = LoadAeroTables()
    T_trans_raw = readtable("B0000.xlsx");
    T_rot_raw = readtable("xpitch_B0000.xlsx");
    T_thrust_raw = readtable("thrust_file");

    T_trans = T_trans_raw(:,2:end);
    T_rot = T_rot_raw(:,2:end);
    T_thrust = T_thrust_raw(:,2:end);
end