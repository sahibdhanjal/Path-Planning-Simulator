# set terminal png transparent nocrop enhanced size 450,320 font "arial,8" 
# set output 'histograms.2.png'
set boxwidth 0.9 absolute
set style fill   solid 1.00 border lt -1
set key inside right top vertical Right noreverse noenhanced autotitle nobox
set style histogram clustered gap 1 title textcolor lt -1
set datafile missing '-'
set style data histograms
set xtics border in scale 0,0 nomirror rotate by -45  autojustify
set xtics  norangelimit
set xtics   ()
set title "US immigration from Northern Europe\nPlot selected data columns as histogram of clustered boxes" 
set yrange [ 0.00000 : 300000. ] noreverse nowriteback
x = 0.0
i = 22
## Last datafile plotted: "immigration.dat"
plot 'immigration.dat' using 6:xtic(1) ti col, '' u 12 ti col, '' u 13 ti col, '' u 14 ti col