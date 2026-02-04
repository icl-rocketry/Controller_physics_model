function [aerosplinefits] = LoadAeroTables()
    
    %ROCKET
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

    %GRIDFINS
    CL_alpha_table = readmatrix("honeycomb-clalpha.xlsx");
    CD_alpha_table = readmatrix("honeycomb-cdalpha.xlsx");
    
    %get rid of random values at the end
    CL_alpha_table = CL_alpha_table(2:90, :);

    %1D spline fits
    %to calcuate value at a point: Clval = ppval(Clfit, alpha);
    %do not exceed +- 50 degrees aoa

    Clalphaextended = [-flip(CL_alpha_table(:,1)); CL_alpha_table(:,1)];
    Clextended = [flip(CL_alpha_table(:,2)); CL_alpha_table(:,2)];

    Cdalphaextended = [-flip(CD_alpha_table(:,1)); CD_alpha_table(:,1)];
    Cdextended = [flip(CD_alpha_table(:,2)); CD_alpha_table(:,2)];

    CLfit_spline = spline(Clalphaextended, Clextended);
    CDfit_spline = spline(Cdalphaextended, Cdextended);

    %[CA_xpitch_spline, CN_xpitch_spline, CMy_xpitch_spline, CA_static_spline, CN_static_spline, CMy_static_spline, CLfit, CDfit]

    aerosplinefits = struct("CA_xpitch", CA_xpitch_spline, ...
        "CN_xpitch", CN_xpitch_spline, ...
        "CMy_xpitch", CMy_xpitch_spline, ...
        "CA_static", CA_static_spline, ...
        "CN_static", CN_static_spline, ...
        "CLfit", CLfit_spline, ...
        "CDfit", CDfit_spline);  

end