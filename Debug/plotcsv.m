Location = readmatrix('e.csv')
col1 = Location(:, 1);
col2 = Location(:, 2);
Location1 = readmatrix('m.csv')
col3 = Location1(:, 1);
col4 = Location1(:, 2);
plot(col1, col2, 'r-o', col3, col4, 'b-+' )