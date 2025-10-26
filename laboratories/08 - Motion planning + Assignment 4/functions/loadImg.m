function [map, initial_state, final_state, img] = loadImg(img_name)

switch img_name
    
    case '01'
        initial_state = [1, 12];
        final_state = [28, 12];

    case '02'
        initial_state = [1, 8];
        final_state = [27, 28];

    case '03'
        initial_state = [1, 1];
        final_state = [1, 30];

    case '04'
        initial_state = [1, 8];
        final_state = [19, 19];

    case 'test'
        initial_state = [1, 8];
        final_state = [27, 28];

    otherwise
        error('Unknown input map')

end

initial_state = flip(initial_state);
final_state = flip(final_state);

img = imread(['assets/original/' img_name '.png']);
map = rgb2gray(imread(['assets/resized/' img_name '.png'])) >= 0.95 * 255;
map(initial_state(1), initial_state(2)) = 1;
map(final_state(1), final_state(2)) = 1;

end

