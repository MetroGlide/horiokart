#!/bin/bash


# デフォルトのトピックリストファイル名
default_topics_file="record_topics.txt"

# デフォルトの再生bagディレクトリ名
default_bag_dir="./"

# コマンドライン引数の解析

# -d: bagディレクトリ名, -f: トピックリストファイル, -s: use_sim_time
while getopts ":d:f:s" opt; do
  case $opt in
    d)
      bag_dir="$OPTARG"
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

# bagディレクトリ名が指定されていない場合はカレントディレクトリ
if [ -z "$bag_dir" ]; then
  bag_dir="$default_bag_dir"
fi


# トピックリストからトピックを配列で読み込む
topics=()
while IFS= read -r line; do
  if [ -n "$line" ]; then
    topics+=("$line")
  fi
done < "$topics_file"

# ros2 bag playコマンドのビルド
play_cmd="ros2 bag play \"$bag_dir\""
if [ "$use_sim_time" = true ]; then
  play_cmd+=" --clock"
fi

# --topics オプションでトピックを指定
if [ ${#topics[@]} -gt 0 ]; then
  play_cmd+=" --topics"
  for topic in "${topics[@]}"; do
    play_cmd+=" $topic"
  done
fi

# トピック名をecho
echo "----------"
echo "Playing topics:"
for topic in "${topics[@]}"; do
  echo "  $topic"
done
echo "----------"

# playコマンドを実行
eval "$play_cmd"
