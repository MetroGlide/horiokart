#!/bin/bash

# デフォルトのトピックリストファイル名
default_topics_file="record_topics.txt"

# デフォルトの出力ファイル名生成関数
generate_filename() {
    date +"%Y%m%d_%H%M%S"
}

# コマンドライン引数の解析
while getopts ":o:f:s" opt; do
  case $opt in
    o)
      output_file="$OPTARG"
      ;;
    f)
      topics_file="$OPTARG"
      ;;
    s)
      use_sim_time=true
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
  esac
done

# トピックリストファイルが指定されていない場合はデフォルトのファイル名を使用
if [ -z "$topics_file" ]; then
    topics_file="$default_topics_file"
fi

# ファイル名が指定されていない場合は現在時刻から生成
if [ -z "$output_file" ]; then
    output_file="$(generate_filename).bag"
fi

# record_topics.txtからトピックを読み込む
topics=()
while IFS= read -r line; do
    if [ -n "$line" ]; then
        topics+=("$line")
    fi
done < "$topics_file"

# rosbag2のrecordコマンドのビルド
record_cmd="ros2 bag record"
if [ "$use_sim_time" = true ]; then
    record_cmd+=" --use-sim-time"
fi

# トピックをコマンドに追加
for topic in "${topics[@]}"; do
    record_cmd+=" $topic"
done

# トピック名をecho
echo "----------"
echo "Recording topics:"
for topic in "${topics[@]}"; do
    echo "  $topic"
done
echo "----------"

# recordコマンドを実行
eval "$record_cmd -o $output_file"
