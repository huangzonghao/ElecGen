function process_bmp(filename)
if ~isfile(filename)
    fprintf('Error: cannot find %s, return', filename);
end

[path, basename ,ext] = fileparts(filename);
img = imread(filename);
img_size = size(img);
img = uint8(rescale(img, 0, 255));
img = reshape(img, img_size);
imwrite(img, strcat(basename , '_processed.bmp'));
end
