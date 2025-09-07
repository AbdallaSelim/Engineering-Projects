function animate_scara(q1_vals, q2_vals, d3_vals, q4_vals, save_video)
    % Link lengths
    L1 = 0.2;   
    L2 = 0.3;
    L3 = 0.2;
    L5 = 0.05;
    
    N = length(q1_vals);
    assert(all([length(q2_vals), length(d3_vals), length(q4_vals)] == N), ...
        'All joint vectors must be the same length');

    % Setup figure
    fig = figure;
    axis equal;
    axis([-0.2 0.6 -0.6 0.6 0 0.6]);
    grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(3);
    hold on;
    title('SCARA Robot Animation with Coordinate Frames');

    % World frame
    quiver3(0,0,0, 0.1,0,0, 'r', 'LineWidth', 1.5);
    quiver3(0,0,0, 0,0.1,0, 'g', 'LineWidth', 1.5);
    quiver3(0,0,0, 0,0,0.1, 'b', 'LineWidth', 1.5);

    % Create placeholders for links
    h_links(1) = plot3([0 0],[0 0],[0 0],'k','LineWidth',2);  % base to P0
    h_links(2) = plot3([0 0],[0 0],[0 0],'b','LineWidth',2);  % P0 to P1
    h_links(3) = plot3([0 0],[0 0],[0 0],'g','LineWidth',2);  % P1 to P2
    h_links(4) = plot3([0 0],[0 0],[0 0],'r','LineWidth',2);  % P2 to P3
    h_links(5) = plot3([0 0],[0 0],[0 0],'m--','LineWidth',1.5); % P3 to P4

    % Create trace line
    trace = animatedline('Color','m','LineStyle','--','LineWidth',1);

    % Setup video recording if requested
    if nargin > 4 && save_video
        v = VideoWriter('scara_animation.mp4', 'MPEG-4');
        v.Quality = 100;
        v.FrameRate = 20;
        open(v);
    else
        v = [];
    end

    % Animation loop
    for i = 1:N
        q1 = q1_vals(i);
        q2 = q2_vals(i);
        d3 = d3_vals(i);
        q4 = q4_vals(i);

        % Rotation matrices
        R10 = rotz(q1);
        R21 = rotz(q2);
        R32 = rotz(q4);

        % Positions
        P0 = [0; 0; L1];
        P1 = P0 + R10 * [L2; 0; 0];
        P2 = P1 + R10*R21 * [L3; 0; 0];
        P3 = P2 + [0; 0; -d3];
        P4 = P3 + R10*R21*R32 * [0; 0; L5];

        % Update links
        set(h_links(1), 'XData', [0 P0(1)], 'YData', [0 P0(2)], 'ZData', [0 P0(3)]);
        set(h_links(2), 'XData', [P0(1) P1(1)], 'YData', [P0(2) P1(2)], 'ZData', [P0(3) P1(3)]);
        set(h_links(3), 'XData', [P1(1) P2(1)], 'YData', [P1(2) P2(2)], 'ZData', [P1(3) P2(3)]);
        set(h_links(4), 'XData', [P2(1) P3(1)], 'YData', [P2(2) P3(2)], 'ZData', [P2(3) P3(3)]);
        set(h_links(5), 'XData', [P3(1) P4(1)], 'YData', [P3(2) P4(2)], 'ZData', [P3(3) P4(3)]);

        % Update trace
        addpoints(trace, P4(1), P4(2), P4(3));

        % Plot coordinate axes at end-effector
        delete(findall(gca, 'Tag', 'eef_axis')); % remove old ones
        Rtool = R10 * R21 * R32;
        scale = 0.05;
        quiver3(P4(1), P4(2), P4(3), Rtool(1,1)*scale, Rtool(2,1)*scale, Rtool(3,1)*scale, ...
                'r', 'LineWidth', 1, 'Tag', 'eef_axis');
        quiver3(P4(1), P4(2), P4(3), Rtool(1,2)*scale, Rtool(2,2)*scale, Rtool(3,2)*scale, ...
                'g', 'LineWidth', 1, 'Tag', 'eef_axis');
        quiver3(P4(1), P4(2), P4(3), Rtool(1,3)*scale, Rtool(2,3)*scale, Rtool(3,3)*scale, ...
                'b', 'LineWidth', 1, 'Tag', 'eef_axis');

        title(sprintf('SCARA Robot - Step %d/%d', i, N));
        drawnow;

        % Write video frame
        if ~isempty(v)
            frame = getframe(fig);
            writeVideo(v, frame);
        end

        pause(0.05);
    end

    if ~isempty(v)
        close(v);
        disp('Video saved as scara_animation.mp4');
    end
end
