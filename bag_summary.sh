#!/bin/bash

# 引数からバッグファイルのディレクトリパスを取得
bag_directory="$1"

# 結果を保存するテキストファイルのパス
output_file="bag_info.txt"

# 引数が与えられなかった場合のエラーチェック
if [ -z "$bag_directory" ]; then
  echo "バッグファイルのディレクトリパスを引数として指定してください。"
  exit 1
fi

# テキストファイルを初期化
> "$output_file"

# バッグファイルのリストを取得
bag_files=$(find "$bag_directory" -name "*.mcap")

# sort
bag_files=$(echo "$bag_files" | sort)

# テキストファイルを初期化
> "$output_file"

# バッグファイルごとに情報を取得し、テキストファイルに書き込む
for bag_file in $bag_files; do
  file_size=$(stat -c %s "$bag_file")  # ファイルサイズ (バイト)
  topic_count=$(ros2 bag info "$bag_file" | grep "topics" | awk '{print $2}')  # トピック数

  # テキストファイルに結果を追加
  echo "Bag File: $bag_file" >> "$output_file"
  echo "File Size (bytes): $file_size" >> "$output_file"
  echo "File Size (MB): $(echo "scale=2; $file_size / 1024 / 1024" | bc)" >> "$output_file"
  echo "File Size (GB): $(echo "scale=2; $file_size / 1024 / 1024 / 1024" | bc)" >> "$output_file"
#   echo "Topic Count: $topic_count" >> "$output_file"
  echo "-----------------------------------------" >> "$output_file"
done

# 結果を表示
cat "$output_file"

echo "処理が完了しました。結果は $output_file に保存されました。"
