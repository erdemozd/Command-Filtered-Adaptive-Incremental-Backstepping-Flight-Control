function elevator = airliner_mesh(parent,color)
%AIRLINER_MESH Original generic narrow-body airliner for MATLAB graphics.
% Original aircraft geometry; TU Delft artwork is credited in assets/branding.
% x forward, y lateral, z upward. Dimensions are visual proportions only.
x = linspace(-20,20,220);
stations = [-20 -19 -17 -14 -10 9 13 15 17 18.5 19.5 20];
radii = [0.03 0.25 0.7 1.3 1.85 1.85 1.85 1.76 1.45 1.02 0.57 0.03];
r = interp1(stations,radii,x,'pchip');
zc = interp1(stations,[0.65 0.6 0.45 0.2 0 0 0 0 -0.06 -0.18 -0.3 -0.35],x,'pchip');
shell(parent,x,r,zc,0,[0.93 0.95 0.97]);
wingColor = [0.72 0.77 0.82]; dark = [0.055 0.10 0.15];
% Smooth belly/wing-root fairing rather than wings intersecting a bare tube.
[fx,fy,fz] = ellipsoid(-1.0,0,-1.1,5.0,2.3,0.9,32);
surf(fx,fy,fz,'Parent',parent,'FaceColor',[.84 .87 .90],'EdgeColor','none');
logoPath = fullfile(fileparts(mfilename('fullpath')),'assets','branding','tu_delft.png');
[logo,~,logoAlpha] = imread(logoPath);
for side = [-1 1]
    % Tapered wings with thickness, sweep, dihedral, and small winglets.
    wing(parent,side,[1.4 7.5 17],[4 .3 -6.7],[8.5 5.6 1.8],[-.45 .0 .65],wingColor);
    foil(parent,[-6.7 side*17 .65; -7.3 side*17.7 2.85; ...
        -8.2 side*17.7 2.85; -8.5 side*17 .65],0.045,color);
    line([-3.2 -4.5 -8.1],side*[2 8 16],[-0.14 0.25 .84], ...
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
    for blade = linspace(0,2*pi,25)
        ba = blade+[0 .12 .25 .10]; br = [.24 .79 .79 .24];
        panel(parent,[4.06*ones(4,1),side*5.9+br(:).*cos(ba(:)), ...
            -2.05+br(:).*sin(ba(:))],[.32 .38 .44]);
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
    % Six wraparound cockpit panes, individual metal frames, and wipers.
    cockpitX = {[17.2 18.1 18.0 17.1], [15.8 17.05 16.95 15.8], ...
        [14.9 15.65 15.65 14.9]};
    for pane = 1:3
        vx = cockpitX{pane}(:); phi = [.37 .38 .91 .91].';
        rr = interp1(stations,radii,vx,'pchip')+.035;
        zz = interp1(x,zc,vx,'pchip');
        v = [vx, side*rr.*cos(phi), zz+rr.*sin(phi)];
        % Tessellate the glass onto the curved skin; a single flat polygon
        % would sink into the fuselage and disappear behind its white hull.
        [uu,vv] = meshgrid(linspace(0,1,12));
        weights = cat(3,(1-uu).*(1-vv),uu.*(1-vv),uu.*vv,(1-uu).*vv);
        gx = sum(weights.*reshape(vx,1,1,4),3);
        gp = sum(weights.*reshape(phi,1,1,4),3);
        gr = interp1(stations,radii,gx,'pchip')+.055;
        gz = interp1(x,zc,gx,'pchip');
        surf(gx,side*gr.*cos(gp),gz+gr.*sin(gp),'Parent',parent, ...
            'FaceColor',[.055 .15 .23],'EdgeColor','none', ...
            'FaceLighting','gouraud','SpecularStrength',.7);
        line(v([1:4 1],1),v([1:4 1],2),v([1:4 1],3), ...
            'Parent',parent,'Color',[.52 .60 .66],'LineWidth',.65);
        if pane==1
            w = .7*v(1,:)+.3*v(4,:);
            e = .35*v(2,:)+.65*v(3,:);
            line([w(1) e(1)],[w(2) e(2)],[w(3) e(3)]+.025, ...
                'Parent',parent,'Color',[.04 .05 .06],'LineWidth',.8);
        end
    end
    % Door outlines and a simple original accent strip.
    for dx = [-12.5 -1.4 1.0 13.6]
        vz = [-.65 -.65 1.15 1.15 -.65]; vx = dx+[-.23 .23 .23 -.23 -.23];
        rr = interp1(stations,radii,vx,'pchip'); zz = interp1(x,zc,vx,'pchip');
        vy = side*(sqrt(max(0,rr.^2-(vz-zz).^2))+0.025);
        line(vx,vy,vz,'Parent',parent,'Color',[0.5 0.57 0.63],'LineWidth',0.5);
    end
    sx = linspace(-12.5,14,100); sr = interp1(stations,radii,sx,'pchip');
    sz = interp1(x,zc,sx,'pchip');
    line(sx,side*sqrt(sr.^2-(-0.2-sz).^2)*1.005,-0.2*ones(size(sx)), ...
        'Parent',parent,'Color',color,'LineWidth',1.3);
    % Official logo, retaining its full artwork and clear-space margins.
    % Cylindrical decal preserves aspect ratio in fuselage surface distance.
    lx = linspace(6,12,70);
    if side>0, lx=fliplr(lx); end % Read correctly from either side.
    phi = linspace(.50,.50-6/1.85*size(logo,1)/size(logo,2),40).';
    [xx,pp] = meshgrid(lx,phi);
    rr = interp1(stations,radii,xx,'pchip')+.035;
    zz = interp1(x,zc,xx,'pchip');
    surf(xx,side*rr.*cos(pp),zz+rr.*sin(pp),'Parent',parent, ...
        'CData',logo,'FaceColor','texturemap','EdgeColor','none', ...
        'AlphaData',double(logoAlpha)/255,'AlphaDataMapping','none', ...
        'FaceAlpha','texturemap','FaceLighting','none');
    % Navigation lights at the original wingtip geometry.
    [nx,ny,nz] = sphere(8);
    navColor = [1 .12 .08]; if side>0, navColor=[.12 .9 .4]; end
    surf(-7.0+.09*nx,side*17+.09*ny,.72+.09*nz,'Parent',parent, ...
        'FaceColor',navColor,'EdgeColor','none','FaceLighting','none');
end
foil(parent,[-12.7 0 1.1; -16.7 0 6.8; -18.8 0 6.8; -18.2 0 0.9],0.13,color);
% Small dorsal antennae and tail rudder seam.
for ax = [-7 8]
    foil(parent,[ax 0 1.85;ax-.25 0 2.5;ax-.65 0 2.5;ax-.5 0 1.85],.045,[.7 .75 .8]);
end
line([-17.9 -18.3],[.145 .145],[1.3 6.3],'Parent',parent,'Color',color*.65,'LineWidth',.6);
elevator = hgtransform('Parent',parent);
% animate_airbus_controllers sets this transform's hinge to [-13 0 0.5].
for side = [-1 1]
    foil(elevator,[-1.4 side*0.8 0; -3.7 side*6.5 0.6; ...
        -4.6 side*6.5 0.6; -3.4 side*0.8 0],0.06,[0.55 0.62 0.69]);
end
end

function wing(parent,side,span,leading,chord,height,color)
% Lofted airfoil with rounded nose, camber, and a thin trailing edge.
y = linspace(span(1),span(end),65).';
u = (1-cos(linspace(0,pi,64)))/2;
c = interp1(span,chord,y,'pchip'); le = interp1(span,leading,y,'pchip');
z = interp1(span,height,y,'pchip');
profile = 5*.105*(.2969*sqrt(u)-.1260*u-.3516*u.^2+.2843*u.^3-.1036*u.^4);
xx = le-c*u; yy = side*repmat(y,1,numel(u));
camber = .018*sin(pi*u);
for surfaceSide = [-1 1]
    zz = z+c*(camber+surfaceSide*profile);
    surf(xx,yy,zz,'Parent',parent,'FaceColor',color,'EdgeColor','none', ...
        'FaceLighting','gouraud','SpecularStrength',.3);
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
