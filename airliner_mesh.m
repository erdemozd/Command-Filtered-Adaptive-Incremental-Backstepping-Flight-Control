function elevator = airliner_mesh(parent,color)
%AIRLINER_MESH Original generic narrow-body airliner for MATLAB graphics.
% No imported mesh, manufacturer drawings, logos, or airline liveries.
% x forward, y lateral, z upward. Dimensions are visual proportions only.
x = linspace(-20,20,150);
stations = [-20 -19 -17 -14 -10 9 13 15 17 18.5 19.5 20];
radii = [0.03 0.25 0.7 1.3 1.85 1.85 1.8 1.65 1.3 0.88 0.42 0.03];
r = interp1(stations,radii,x,'pchip');
zc = interp1(stations,[0.65 0.6 0.45 0.2 0 0 0 0 -0.06 -0.18 -0.3 -0.35],x,'pchip');
shell(parent,x,r,zc,0,[0.93 0.95 0.97]);
wingColor = [0.72 0.77 0.82]; dark = [0.055 0.10 0.15];
for side = [-1 1]
    % Tapered wings with thickness, sweep, dihedral, and small winglets.
    foil(parent,[4 side*1.5 -0.45; -1 side*8 0; -7.5 side*17 0.9; ...
        -9.6 side*17 0.9; -5 side*7 -0.1; -4.5 side*1.5 -0.45],0.17,wingColor);
    foil(parent,[-7.5 side*17 0.9; -8.5 side*17.6 2.6; ...
        -9.4 side*17.6 2.6; -9.6 side*17 0.9],0.05,color);
    line([-4.2 -6.0 -9.1],side*[2 8 16],[-0.24 0.17 1.01], ...
        'Parent',parent,'Color',[0.40 0.47 0.53],'LineWidth',0.6);
    % Fixed tailplane; the separate rear section animates with the elevator.
    foil(parent,[-12 side*0.8 0.5; -15.8 side*6.5 1.1; ...
        -16.7 side*6.5 1.1; -14.4 side*0.8 0.5],0.10,wingColor);
    % Engine pylon and rounded nacelle with a recessed fan and exhaust.
    foil(parent,[3.2 side*5.9 -0.1; 1.0 side*5.9 -0.1; ...
        0.7 side*5.9 -2.0; 3.0 side*5.9 -2.0],0.12,wingColor);
    ex = linspace(-0.6,4.5,50);
    er = interp1([-0.6 0.2 1.8 3.8 4.5],[0.65 1.05 1.18 1.12 1.0],ex,'pchip');
    shell(parent,ex,er,-2.05*ones(size(ex)),side*5.9,[0.83 0.87 0.91]);
    shell(parent,[4.5 4.35 4.05],[1 .89 .82],[-2.05 -2.05 -2.05],side*5.9,[0.48 0.55 0.62]);
    disk(parent,4.02,side*5.9,-2.05,0.82,dark);
    for blade = linspace(0,2*pi,19)
        line([4.06 4.06],side*5.9+[0.22*cos(blade) .78*cos(blade+0.15)], ...
            -2.05+[0.22*sin(blade) .78*sin(blade+0.15)], ...
            'Parent',parent,'Color',[0.32 0.38 0.44],'LineWidth',0.65);
    end
    shell(parent,[4.07 4.30],[0.23 0.02],[-2.05 -2.05],side*5.9,[0.56 0.61 0.66]);
    shell(parent,[-0.9 -0.55],[0.4 .62],[-2.05 -2.05],side*5.9,[0.27 0.31 0.35]);
    disk(parent,-0.92,side*5.9,-2.05,0.39,dark);
    % Rounded cabin windows fitted to the fuselage surface.
    for wx = -11:0.85:13
        ang = linspace(0,2*pi,16).';
        vx = wx+0.14*cos(ang); vz = 0.62+0.21*sin(ang);
        rr = interp1(stations,radii,vx,'pchip'); zz = interp1(x,zc,vx,'pchip');
        vy = side*(sqrt(max(0,rr.^2-(vz-zz).^2))+0.018);
        panel(parent,[vx vy vz],dark);
    end
    % Cockpit panes follow the tapered nose surface.
    for cx = [15.1 16.2]
        vx = [cx-.4; cx+.4; cx+.35; cx-.35]; vz = [0.72;0.68;1.14;1.20];
        rr = interp1(stations,radii,vx,'pchip'); zz = interp1(x,zc,vx,'pchip');
        vy = side*(sqrt(max(0,rr.^2-(vz-zz).^2))+0.025);
        panel(parent,[vx vy vz],dark);
    end
    % Door outlines and a simple original accent strip.
    for dx = [-12.5 13.6]
        vz = [-.65 -.65 1.15 1.15 -.65]; vx = dx+[-.23 .23 .23 -.23 -.23];
        rr = interp1(stations,radii,vx,'pchip'); zz = interp1(x,zc,vx,'pchip');
        vy = side*(sqrt(max(0,rr.^2-(vz-zz).^2))+0.025);
        line(vx,vy,vz,'Parent',parent,'Color',[0.5 0.57 0.63],'LineWidth',0.5);
    end
    sx = linspace(-12.5,14,100); sr = interp1(stations,radii,sx,'pchip');
    sz = interp1(x,zc,sx,'pchip');
    line(sx,side*sqrt(sr.^2-(-0.2-sz).^2)*1.005,-0.2*ones(size(sx)), ...
        'Parent',parent,'Color',color,'LineWidth',1.3);
end
foil(parent,[-12.7 0 1.1; -16.7 0 6.8; -18.8 0 6.8; -18.2 0 0.9],0.13,color);
elevator = hgtransform('Parent',parent);
% animate_airbus_controllers sets this transform's hinge to [-13 0 0.5].
for side = [-1 1]
    foil(elevator,[-1.4 side*0.8 0; -3.7 side*6.5 0.6; ...
        -4.6 side*6.5 0.6; -3.4 side*0.8 0],0.06,[0.55 0.62 0.69]);
end
end

function panel(parent,vertices,color)
patch('Parent',parent,'Vertices',vertices,'Faces',1:size(vertices,1), ...
    'FaceColor',color,'EdgeColor','none','FaceLighting','gouraud', ...
    'AmbientStrength',0.5,'DiffuseStrength',0.7,'SpecularStrength',0.18);
end

function shell(parent,x,r,z,y,color)
a = linspace(0,2*pi,64).';
surf(repmat(x,numel(a),1),y+cos(a)*r,z+sin(a)*r,'Parent',parent, ...
    'FaceColor',color,'EdgeColor','none','FaceLighting','gouraud', ...
    'AmbientStrength',0.45,'DiffuseStrength',0.65,'SpecularStrength',0.25);
end

function disk(parent,x,y,z,r,color)
a = linspace(0,2*pi,48).';
panel(parent,[x*ones(size(a)), y+r*cos(a), z+r*sin(a)],color);
end

function foil(parent,v,thickness,color)
% Closed thin solid; fins/pylons extrude laterally, wings vertically.
if max(v(:,2))-min(v(:,2))<0.01, offset = [0 thickness 0]; else, offset = [0 0 thickness]; end
panel(parent,v+offset,color); panel(parent,flipud(v-offset),color*0.92);
for k=1:size(v,1)
    q = mod(k,size(v,1))+1;
    panel(parent,[v(k,:)+offset;v(q,:)+offset;v(q,:)-offset;v(k,:)-offset],color*0.8);
end
end
