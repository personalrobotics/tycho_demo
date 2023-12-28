while getopts 'aopr' flag; do
	case "${flag}" in
		o) echo "o" ;;
		r) echo "r" ;;
		a) echo "a" ;;
		p) echo "p" ;;
		*) echo "unrecognized" ;;
	esac
done