POLICIES="ccil naive"
DATAS="50 100 150 200"
for policy in $POLICIES; do
    for num in $DATAS; do
        name="${policy}_${num}"
        python tycho_demo/parse_logs.py tycho_demo/logs/coin_data_ablation/$name/* ~/addon/ablation/data_ablation/coin_$name.pkl -b
    done
done