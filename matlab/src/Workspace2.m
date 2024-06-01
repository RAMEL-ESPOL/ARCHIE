syms theta1 theta2 theta3 theta4 theta5 theta6 real
syms d1 d2 d3 d4 real

% DH parameters
test_dh = [
    theta1+pi/2, 0.09, 0.02, pi/2;
    theta2+pi/2, 0.00, 0.20, 0.0 ;
    theta3     , 0.00, 0.02, pi/2;
    theta4     , 0.23,-0.02,-pi/2;
    theta5-pi/2, 0.00, 0.00, pi/2;
    theta6     , 0.04, 0.00, 0.0 ;
];
% Parameter ranges
theta1_range = arr2Rad(linspace(-180,180, 35));
theta2_range = arr2Rad(linspace(-45,45, 35));
theta3_range = arr2Rad(linspace(-65,115, 35));
theta4_range = arr2Rad(linspace(-180,180, 35));
theta5_range = arr2Rad(linspace(-65,115, 35));
theta6_range = arr2Rad(linspace(0,1, 1));
test_map = containers.Map({'theta1', 'theta2', 'theta3','theta4','theta5','theta6'}, ...
    {theta1_range, theta2_range, theta3_range, theta4_range, theta5_range, theta6_range}); 
% Workspace plotting function
plot3dworkspace(test_dh, test_map, @get_alternative_dh_transform)


function out = arr2Rad(A)
    out = arrayfun(@(angle) deg2rad(angle), A);
end

function T = get_alternative_dh_transform(theta, d, a, alpha)
T = [cos(theta) -cos(alpha)*sin(theta) sin(alpha)*sin(theta) a*cos(theta)
     sin(theta) cos(alpha)*cos(theta) -sin(alpha)*sin(theta) a*sin(theta)
     0 sin(alpha) cos(alpha) d
     0 0 0 1];
end

function plot3dworkspace(dh_parameters, parameter_ranges, dh_transform_fn, verbose)
%PLOT3DWORKSPACE Plots a 3D representation of the reachable workspace of a robot.
%
% Syntax:
%   plot3dworkspace(dh_parameters, parameter_ranges, dh_transform_fn, verbose)
%
% Inputs:
%   dh_parameters: A matrix containing the Denavit-Hartenberg parameters.
%                  Each row represents one link, and the columns are 
%                  [a,alpha,d,theta] for each link.
%
%   parameter_ranges: A containers.Map object where the keys are the names 
%                     of the parameters (as strings), and the values are arrays 
%                     of the values that each parameter can take.
%
%   dh_transform_fn: (Optional) Function handle to a function that computes the 
%                    Denavit-Hartenberg transformation matrix. This 
%                    function should accept four arguments corresponding 
%                    to the four Denavit-Hartenberg parameters (a,alpha,d,theta).
%                    Default: @get_DH_matrix.
%
%   verbose: (Optional) A boolean flag that, when true, triggers the function to 
%            display various calculation details like the transformation matrix 
%            and more. Default: false.
%
% Outputs:
%   No outputs. The function directly plots the reachable workspace.
%

    arguments
        dh_parameters
        parameter_ranges
        dh_transform_fn = @get_DH_matrix
        verbose = false % Whether to log various calculation details like the transformation matrix and more.
    end

    validate_inputs(dh_parameters, parameter_ranges);
    [x,y,z] = get_xyz_expressions(dh_parameters, dh_transform_fn, verbose);
    

    % Get parameter grid
    if verbose
        ranges = values(parameter_ranges);        
        lengths = cellfun(@length, ranges);        
        product_of_lengths = prod(lengths);
        fprintf('allocating %d possible DH parameter combinations...\n', product_of_lengths);
    end

    keys = parameter_ranges.keys;
    param_values = parameter_ranges.values(keys);
    [param_grid{1:numel(param_values)}] = ndgrid(param_values{:});
    
    % Reshape into value arrays in map
    param_value_map = containers.Map;
    for i = 1:numel(param_grid)
        param_value_map(keys{i}) = reshape(param_grid{i}, 1, []);
        param_grid{i} = 0;
    end
    

    if verbose
        disp('calculating positions from expressions...');
    end
    % Calculate x,y,z positions
    positions = cell(3,1);
    expressions = {x,y,z};
    for i = 1:3
        % Get variables from expression
        expr = expressions{i};
        sym_cell = symvar(expr);
        keys_cell = arrayfun(@char, sym_cell, 'UniformOutput', false);
        % Convert from symbolic to numerical functions for faster evaluation
        pos_func = matlabFunction(expr, 'Vars', sym_cell);
        func_params = values(param_value_map, keys_cell);
        positions{i} = pos_func(func_params{:});
    end
    
    % Plot points
    figure(1)
    plot3(positions{:},'.');
    title('Reachable Workspace')
    xlabel('X') 
    ylabel('Y')
    zlabel('Z')
    axis equal
    grid on
end

function validate_inputs(dh_parameters, parameter_ranges)
    % validate_inputs Validates the inputs to the plotting functions.
    %
    % This function checks if the given DH parameters are a symbolic matrix and if the parameter ranges are 
    % a valid containers.Map object with each value as a numeric array. Also checks for symbol consistency
    % between dh_parameters and parameter_ranges.
    %
    % Parameters:
    %   dh_parameters : A symbolic matrix that represents the DH parameters of the robot.
    %   
    %   parameter_ranges : A containers.Map that represents the range of each DH parameter.
    %


    % Check DH parameters type
    if ~isa(dh_parameters, 'sym') || ~ismatrix(dh_parameters)
        error('dh_parameters must be a symbolic matrix')
    end

    % Validate the size of DH parameters
    [~, cols] = size(dh_parameters);
    if cols ~= 4
        error('dh_parameters must be a n-by-4 matrix for n links each with 4 DH parameters.')
    end
    
    % Validate that parameter_ranges is a containers.Map
    if ~isa(parameter_ranges, 'containers.Map')
        error('parameter_ranges must be a containers.Map')
    end

    % Validate that each value in b is a triple of numbers
    values_in_params = parameter_ranges.values();
    for i = 1:length(values_in_params)
        if ~isa(values_in_params{i}, 'double')
            error('Each parameter range must be an array of numbers')
        end
    end
    
    % Find all unique symbols in dh_parameters
    symbols_in_dh = arrayfun(@char, symvar(dh_parameters), 'UniformOutput', false);
    
    % Get all keys in the map ranges
    keys_in_ranges = parameter_ranges.keys();
    extra_dh = setdiff(symbols_in_dh, keys_in_ranges);
    extra_ranges = setdiff(keys_in_ranges, symbols_in_dh);
    
    % Check if the symbols are the same
    if ~isempty(extra_dh) || ~isempty(extra_ranges)
        error(['The symbols in dh_parameters and keys in parameter_ranges must be the same. ' ...
            'Excess in dh_parameters: %s. Excess in parameter_ranges: %s.'], strjoin(extra_dh, ', '), strjoin(extra_ranges, ', ')')
    end
end

function [x, y, z] = get_xyz_expressions(dh_parameters, dh_transform_fn, verbose)
%GET_XYZ_EXPRESSIONS Calculates the x, y, and z positions from Denavit-Hartenberg parameters.
%
% Syntax:
%   [x, y, z] = get_xyz_expressions(dh_parameters, dh_transform_fn, verbose)
%
% Inputs:
%   dh_parameters: A matrix containing the Denavit-Hartenberg parameters.
%                  Each row represents one link, and the columns are 
%                  [a,alpha,d,theta] for each link.
%
%   dh_transform_fn: Function handle to a function that computes the 
%                    Denavit-Hartenberg transformation matrix. This 
%                    function should accept four arguments corresponding 
%                    to the four Denavit-Hartenberg parameters (a,alpha,d,theta).
%
%   verbose: A boolean flag that, when true, triggers the function to 
%            display the final transformation matrix and the expressions 
%            for position.
%
% Outputs:
%   x, y, z: The symbolic expressions for the x, y, and z coordinates of 
%            the end-effector (i.e., the position of the end-effector in 
%            the base frame).
%
    % Calculate transformation matrix from base to end-effector
    T0_ee = eye(4);
    % Apply each transformation
    for i = 1:size(dh_parameters, 1)
        T{i} = dh_transform_fn(dh_parameters(i,1), ...
            dh_parameters(i,2), ...
            dh_parameters(i,3), ...
            dh_parameters(i,4));
        T0_ee = T0_ee*T{i};
    end

    x = T0_ee(1,4);
    y = T0_ee(2,4);
    z = T0_ee(3,4);

    if verbose
        disp('final transformation matrix from base to end-effector:')
        disp(T0_ee)
        disp('expressions for position:')
        fprintf('x = %s\n', char(x))
        fprintf('y = %s\n', char(y))
        fprintf('z = %s\n', char(z))
    end

end