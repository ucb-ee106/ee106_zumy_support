 #!/bin/bash         

#need to install 'pv'
echo "Loading both Mbed and Zumy Files"
echo "You'd better be ready for the possibility of loosing files"
echo ""

NAME="$1"

echo "Loading to " $NAME
cd on_zumy_odroid/
NUMFILES="$(find . -type f | wc -l)"
echo "rsyncing ros code"

rsync -r --delete --stats --human-readable * "zumy@"$NAME".local:" | pv -lep -s $NUMFILES
cd ../

#make a file with the date & time, but it into the zumy for informational purposes
dt=`date '+%d/%m/%Y %H:%M:%S'`
echo "$dt" > "last_loaded.txt"
scp "last_loaded.txt" "zumy@"$NAME".local:"
