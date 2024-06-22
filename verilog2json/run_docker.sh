# 检查参数是否正确
if [ "$#" -lt 2 ]; then
    echo "Usage: $0 <python_script> [script_arguments]"
    exit 1
fi

# ip_intergrator 工程目录
prj_path=/home/doee/Workspace/ip_integrator

# docker image name
docker_image=antlr-env

# python script name
python_script=$1
shift 1

# arguments
script_arguments=$@

# 计算 python_script 的绝对路径
absolute_path=$(realpath "$python_script")

# 计算相对于 prj_path 的相对路径
relative_path=$(realpath --relative-to="$prj_path" "$absolute_path")

# 运行 Python 脚本
docker run --rm -v "$prj_path:/app" "$docker_image" python "/app/$relative_path" $script_arguments

