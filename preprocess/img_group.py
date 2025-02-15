import os
import shutil
import traceback
def move_file(src_path, dst_path, file):
    print('from : ',src_path)
    print('to : ',dst_path)
    try:
        # cmd = 'chmod -R +x ' + src_path
        # os.popen(cmd)
        f_src = os.path.join(src_path, file)
        if not os.path.exists(dst_path):
            os.mkdir(dst_path)
        f_dst = os.path.join(dst_path, file)
        shutil.move(f_src, f_dst)
    except Exception as e:
        print('move_file ERROR: ',e)
        traceback.print_exc()

dir_path="/home/lzh/dataSet/software_demo/lake/block1"
names=os.listdir(dir_path)
sorted_files = sorted(files, key=lambda x: int(re.search(r'\d+', x).group()))
sub_dir_num=10
# for i in range(len(names)//sub_dir_num+ (1 if len(names)%sub_dir_num!=0 else 0)):
cur_path=None
for i in range(len(names)):
    if not os.path.exists(os.path.join(dir_path,str(i//sub_dir_num))) and i%sub_dir_num==0:
        cur_path=os.path.join(dir_path,str(i//sub_dir_num))
        os.mkdir(cur_path)
    move_file(dir_path,cur_path,names[i])


