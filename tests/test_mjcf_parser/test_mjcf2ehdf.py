import os
import csv
import colorama
import math
from pathlib import Path

def assert_files_equal(path1, path2):
    """自定义断言函数，比较两个文件内容"""
    with open(path1) as f1, open(path2) as f2:
        lines1 = f1.readlines()
        lines2 = f2.readlines()
    
    for i, (line1, line2) in enumerate(zip(lines1, lines2), 1):
        if line1 != line2:
            raise AssertionError(
                f"文件 {path1.name} 和 {path2.name} 第 {i} 行不一致:\n"
                f"预期: {line1.rstrip()}\n"
                f"实际: {line2.rstrip()}"
            )
    
    if len(lines1) != len(lines2):
        raise AssertionError(
            f"文件 {path1.name} 和 {path2.name} 行数不同: "
            f"预期 {len(lines1)} 行，实际 {len(lines2)} 行"
        )

def is_float(value):
    """Check if a string can be converted to float."""
    try:
        float(value)
        return True
    except (ValueError, TypeError):
        return False

def assert_csv_files_equal(path1, path2, float_tolerance=1e-6):
    """比较两个CSV文件内容，将数字字段作为float比较"""
    with open(path1, 'r') as f1, open(path2, 'r') as f2:
        csv1 = list(csv.reader(f1))
        csv2 = list(csv.reader(f2))
    
    # 检查行数是否相同
    if len(csv1) != len(csv2):
        raise AssertionError(
            f"CSV文件 {path1.name} 和 {path2.name} 行数不同: "
            f"预期 {len(csv1)} 行，实际 {len(csv2)} 行"
        )
    
    # 逐行比较
    for i, (row1, row2) in enumerate(zip(csv1, csv2), 1):
        # 检查列数是否相同
        if len(row1) != len(row2):
            raise AssertionError(
                f"CSV文件 {path1.name} 和 {path2.name} 第 {i} 行列数不同: "
                f"预期 {len(row1)} 列，实际 {len(row2)} 列"
            )
        
        # 逐列比较
        for j, (val1, val2) in enumerate(zip(row1, row2), 1):
            # 如果两个值都可以转换为float，则按float比较
            if is_float(val1) and is_float(val2):
                float1 = float(val1)
                float2 = float(val2)
                
                # 处理接近零的情况
                if abs(float1) < float_tolerance and abs(float2) < float_tolerance:
                    continue
                
                # 比较相对误差
                relative_diff = abs((float1 - float2) / float1) if float1 != 0 else abs(float2)
                if relative_diff > float_tolerance and abs(float1 - float2) > float_tolerance:
                    raise AssertionError(
                        f"CSV文件 {path1.name} 和 {path2.name} 第 {i} 行第 {j} 列数值不同:\n"
                        f"预期: {float1}\n"
                        f"实际: {float2}\n"
                        f"差异: {abs(float1 - float2)} (相对误差: {relative_diff})"
                    )
            # 否则按字符串比较
            elif val1 != val2:
                raise AssertionError(
                    f"CSV文件 {path1.name} 和 {path2.name} 第 {i} 行第 {j} 列不同:\n"
                    f"预期: {val1}\n"
                    f"实际: {val2}"
                )

def test_mjcfparser():
    dir1 = Path("assets/transfered_robot")
    dir2 = Path("assets/mytransfered_robot")
    
    # 获取两个目录的文件列表
    files1 = [file.relative_to(dir1) for file in dir1.rglob('*') if file.is_file()]
    files2 = [file.relative_to(dir2) for file in dir2.rglob('*') if file.is_file()]
    
    # 检查文件列表是否相同
    assert set(files1) == set(files2), "目录中的文件列表不同"
    
    # 逐个比较文件内容
    for file1 in files1:
        file2 = dir2 / file1
        file1 = dir1 / file1
        if file1.suffix == '.csv':
            assert_csv_files_equal(file1, file2), f"文件 {file1.name} 内容不同"
        else:
            # 对于其他类型的文件，直接比较内容
            if file1.suffix == '.xml':
                assert_files_equal(file1, file2), f"文件 {file1.name} 内容不同"