import pandas as pd

def get_points(obj_sheet):
    num_sub_obj = (len(obj_sheet) - 2) // 2
    sub_obj_points = []
    for i in range(num_sub_obj):
        x = obj_sheet.iloc[1 + i, 2:].dropna().values
        y = obj_sheet.iloc[2 + num_sub_obj + i, 2:].dropna().values
        sub_obj_points.extend(list(zip(x, y)))
    return sub_obj_points

def read_objectives(file_path):
    obj1_sheet = pd.read_excel(file_path, sheet_name='Obj_1_Loc_Point', header=None)
    obj2_sheet = pd.read_excel(file_path, sheet_name='Obj_2_Loc_Point', header=None)
    obj3_sheet = pd.read_excel(file_path, sheet_name='Obj_3_Loc_Point', header=None)
    
    obj1_points = get_points(obj1_sheet)
    obj2_points = get_points(obj2_sheet)
    obj3_points = get_points(obj3_sheet)

    return obj1_points, obj2_points, obj3_points


if __name__ == "__main__":
    obj1_points, obj2_points, obj3_points = read_objectives('data/T_Objectives_latlon.xlsx')
    print(obj1_points)
    print(obj2_points)
    print(obj3_points)
