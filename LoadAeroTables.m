function [static_table, dynamic_table, fin_table] = LoadAeroTables()
    static_table_raw = readtable("B0000.xlsx");
    dynamic_table_raw = readtable("xpitch_B0000.xlsx");
    fin_table_raw = readtable("fin_table");

    static_table = static_table_raw(:,2:end);
    dynamic_table = dynamic_table_raw(:,2:end);
    fin_table = fin_table_raw(:,2:end);
end