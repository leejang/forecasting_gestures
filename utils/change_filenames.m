
root_dir = '/home/leejang/ros_ws/src/forecasting_gestures/script/new_hri_data';

% contain all sub directory list
all_dir_list = dir(root_dir);
all_dir_list(~[all_dir_list.isdir]) = [];
tf = ismember( {all_dir_list.name}, {'.', '..'});
all_dir_list(tf) = [];  %remove current and parent directory.

%sub_dir_info = cell(length(all_dir_list));

total_cnt = 0;

for k = 1 : length(all_dir_list)
  this_dir = all_dir_list(k).name;
  disp(this_dir);
  
  %target_file = dir(fullfile(root_dir, this_dir, '*.jpg'));
  target_dir = dir(fullfile(root_dir, this_dir, '*.jpg'));
  N = natsortfiles({target_dir.name});
  for i = 1: numel(N)
    origin_name = N{i};
    % with full path
    origin_filename = fullfile(root_dir, this_dir, origin_name);

    disp('Original File name:')
    disp(origin_filename);

    target_name = strcat(this_dir, sprintf('_%04d.jpg', i)); 
    target_filename = fullfile(root_dir, this_dir, target_name);

    disp('Target File name:')
    disp(target_filename);

    movefile(origin_filename, target_filename);

    total_cnt = total_cnt + 1;
  end

end

%disp(total_cnt);
