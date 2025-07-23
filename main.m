function main()
    % Start the filter platform GUI
    fprintf('=== Starting Filter Platform ===\n');
    % Add necessary paths
    addpath('core', 'gui', 'simulation', 'Algorithms'); 
    try
        % Launch the GUI
        fprintf('Starting GUI...\n');
        main_window();    
        fprintf('===== platform launched =====\n');
    catch ME
        fprintf('Launch failed: %s\n', ME.message);
    end
end