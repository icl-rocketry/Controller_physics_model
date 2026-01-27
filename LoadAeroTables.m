function [CA_xpitch_spline, CN_xpitch_spline, CMy_xpitch_spline, CA_static_spline, CN_static_spline, CMy_static_spline] = LoadAeroTables()

    static_table_raw = readtable("B0000.xlsx");
    dynamic_table_raw = readtable("xpitch_B0000.xlsx");
    %fin_table_raw = readtable("fin_table");

    static_table_raw = static_table_raw{:,2:19};
    dynamic_table_raw = dynamic_table_raw{:, 2:19};

    %array definitions:
    %xpitch: 
    M_xpitch = dynamic_table_raw(:, 2);
    alpha_xpitch = dynamic_table_raw(:, 3);
    CA_xpitch = dynamic_table_raw(:, 4);
    CN_xpitch = dynamic_table_raw(:,8);
    CMy_xpitch = dynamic_table_raw(:,12);

    %non xpitch:
    M_static = static_table_raw(:, 2);
    alpha_static = static_table_raw(:, 3);
    CA_static = static_table_raw(:, 4);
    CN_static = static_table_raw(:,8);
    CMy_static = static_table_raw(:,12);


    %xpitch fits
    x = [M_xpitch, alpha_xpitch];
    CA_xpitch_spline = tpaps(x', CA_xpitch', 0.8);
    CN_xpitch_spline = tpaps(x', CN_xpitch', 0.8);
    CMy_xpitch_spline = tpaps(x', CMy_xpitch', 0.8);

    %non pitch fits
    x = [M_static, alpha_static];
    CA_static_spline = tpaps(x', CA_static', 0.8);
    CN_static_spline = tpaps(x', CN_static', 0.8);
    CMy_static_spline = tpaps(x', CMy_static', 0.8);

    %to evaluate a particular value: fnval(CA_xpitch_spline, query_point),
    %where query_point = [M, alpha]
    %to plot: fnplt(CA_xpitch_spline)

end