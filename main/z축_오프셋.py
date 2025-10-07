# 입력 파일과 출력 파일의 이름을 지정합니다.
input_filename = 'poses_odom.txt'
output_filename = 'poses_odom_modified.txt'

try:
    # 파일을 읽고 쓰기 모드로 엽니다.
    with open(input_filename, 'r') as infile, open(output_filename, 'w') as outfile:
        # 입력 파일의 각 줄을 순회합니다.
        for line in infile:
            # 공백을 기준으로 줄을 나눕니다.
            parts = line.strip().split()

            # 줄에 12개의 값이 있는지 확인합니다.
            if len(parts) == 12:
                # 12번째 값(인덱스 11)을 부동소수점 숫자로 변환합니다.
                z_value = float(parts[11])
                
                # 그 값에 3을 더합니다.
                new_z_value = z_value + 3
                
                # 수정된 값을 다시 리스트에 넣습니다.
                parts[11] = str(new_z_value)
                
                # 수정된 리스트를 다시 공백으로 구분된 문자열로 합칩니다.
                new_line = ' '.join(parts)
                
                # 출력 파일에 수정된 줄을 씁니다.
                outfile.write(new_line + '\n')
            else:
                # 형식이 맞지 않는 줄은 그대로 씁니다.
                outfile.write(line)

    print(f"'{output_filename}' 파일이 성공적으로 생성되었습니다.")

except FileNotFoundError:
    print(f"오류: '{input_filename}' 파일을 찾을 수 없습니다.")
except Exception as e:
    print(f"오류가 발생했습니다: {e}")