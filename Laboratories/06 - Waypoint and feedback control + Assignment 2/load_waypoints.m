function waypoints = load_waypoints()

waypoints = [
    +0 +0 0;
    +5 +0 1/2;
    +5 +5 5/4;
    -5 -5 1/2;
    -5 +5 0/1;
    +0 +0 0/1;
    +3 +3 3/4;
    -3 +0 3/2;
    +0 -3 1/4;
    +3 +0 1/2;
    +0 +0 3/2
];

waypoints(:, 3) = waypoints(:, 3) * pi;

end

