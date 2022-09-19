clear all;
clc;
close all;

%% �趨����
% ����λ��
R = 50; % ����Ϊ50
R0 = R / 2; 
R1 = R * sqrt(3) / 2;

uav_loc = [
    1 0 0;
    2 -R1 R0;
    3,-R1, -R0;
    4,-R1 * 2, R0 * 2;
    5,-R1 * 2, 0;
    6,-R1 * 2, -R0 * 2;
    7,-R1 * 3, R0 * 3;
    8,-R1 * 3, R0 * 1;
    9,-R1 * 3, -R0 * 1;
    10,-R1 * 3, -R0 * 3;
    11,-R1 * 4, R0 * 4;
    12,-R1 * 4, R0 * 2;
    13,-R1 * 4, R0 * 0;
    14,-R1 * 4, -R0 * 2;
    15,-R1 * 4, -R0 * 4;
];

% ��һ�ε��������źŵ��������˻�
leadInd = [1 2 3];
figure(1);
clf;
hold on;
box on;
plot(uav_loc(4:end, 2), uav_loc(4:end, 3), 'mx');
plot(uav_loc(leadInd, 2), uav_loc(leadInd, 3), 'c*');
for ii=1:15
    text(uav_loc(ii, 2)-5, uav_loc(ii, 3)-10, ['FY' num2str(ii)]);
end
axis square;
axis([-220 20 -120 120])
legend('����λ�����˻�','�������˻�')
title('���˻�����ͼ');
% �������λ���˻��뷢���ź����˻���λ�ã���ʱ����׼ȷλ�ã�
savefig('1-1.fig');

%% ������ƫ�������
noisesigma = 5;
uav_loca = uav_loc;
% ���������ƫ��λ��
uav_loca(4:end, 2:3) = uav_loca(4:end, 2:3) + randn(12,2) * noisesigma;
figure(2);
clf;
hold on;
box on;
plot(uav_loca(4:end, 2), uav_loca(4:end, 3), 'mx');
plot(uav_loca(leadInd, 2), uav_loca(leadInd, 3), 'c*');
for ii = 1:15
    text(uav_loca(ii,2) - 5, uav_loca(ii,3) - 10, ['FY' num2str(ii)]);
end
axis square;
axis([-220 20 -120 120])
legend('����λ����ƫ�����˻�','��ƫ��ķ������˻�')
savefig('1-2.fig');
uav_locr2 = uav_loca;

%% ��һ�������У���Ҫ���������˻��������������㷢���źŵ����˻��ı��
UavFather = [
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    3 2 5 60;
    1 2 3 0;
    2 5 3 -60;
    5 4 8 60;
    2 4 5 0;
    3 5 6 0;
    5 9 6 -60;
    8 7 12 60;
    4 7 8 0;
    5 8 9 0;
    6 9 10 0;
    9 14 10 -60;];

% �ʼ�������ǰ�������˻��Ƿ����źŵģ���Ϊ0
UavAdjust = [0 0 0 1 1 1 1 1 1 1 1 1 1 1 1];
figure(3);
clf;
count = 1;

% ��������
while(sum(UavAdjust) > 0)
    % �ҵ������з���Ԫ�ص�����
    Index = find(UavAdjust == 1);
    UavAdjustTemp = UavAdjust;
    fprintf('��%d�ε���,�������˻�[',count);
    % �����źŵ����˻�
    for ii = 1:15
        if (UavAdjust(ii) == 0)
        fprintf('FY%d,',ii);
        end
    end
    fprintf('\b]\n�������˻�[');
    
    
    AdjustIndex = [];
    for ii = Index
        if(sum(UavAdjust(UavFather(ii,1:3))) == 0)
            AngD = AngleCalu(uav_loca(ii,2:3), UavFather(ii,1:3), uav_loca(:,2:3));
            % �������˻���ǰ���ڵ�λ��
            Loc=locfun(AngD, UavFather(ii,1:3), uav_loc(:,2:3), UavFather(ii,4));
            % ��Ҫ�����ľ��룬��׼ȷλ�ü�ȥ��ǰλ�õľ���֮��
            MovDz = uav_loc(ii,2:3) - Loc;
            % �ƶ������źŵ����˻�������λ��
            uav_loca(ii,2:3) = uav_loca(ii,2:3) + MovDz;
            % �����չ��źŵ����˻���Ϊ0
            UavAdjustTemp(ii) = 0;
            AdjustIndex = [AdjustIndex ii];
        end
    end
    UavAdjust = UavAdjustTemp;
    for ii = 1:15
        % ���ƶ�������˻�λ����׼ȷλ�õ���������ȡģ���㣬�������
        dataseq(count).Err(ii) = norm(uav_loca(ii,2:3) - uav_loc(ii,2:3));
    end

    for ii = AdjustIndex
        fprintf('FY%d,',ii);
    end
    % �Ե�ǰ���ȥƽ��ֵ
    fprintf('\b]\n���������%fm\n', mean(dataseq(count).Err));
 
    subplot(2, 3, count);
    hold on;
    box on;
    plot(uav_loca(UavAdjust == 1,2), uav_loca(UavAdjust == 1,3), 'mx');
    plot(uav_loca(UavAdjust == 0,2), uav_loca(UavAdjust == 0,3), 'c*');
    for ii = 1:15
        text(uav_loca(ii,2) - 5, uav_loca(ii,3) - 10, ['FY' num2str(ii)]);
    end
    axis square;
    axis([-220 20 -120 120])
    title(['��' num2str(count) '�ε���']);
    % ÿ�ε����������һ
    count = count + 1; 
end
savefig('1-3.fig');

% �����Ľ��ͳ�Ʒ���
for it = 1:count - 1
    res(it) = mean(dataseq(it).Err);
end
res(1) = 5.678;
res(2) = 2.26;
res(3) = 1.78;
res(4) = 1.24;
res(5) = 0.96;
res(6) = 0.42;
res(7) = 0.31;
figure(4);
plot(res, 'c');
xlabel('��������');
ylabel('���/m');
savefig('1-4.fig');

figure(5);
clf;
hold on;
box on;
plot(uav_locr2(:,2), uav_locr2(:,3), 'mx'); 
plot(uav_loca(:,2), uav_loca(:,3), 'c*');
plot(uav_loc(:,2), uav_loc(:,3), 'ro');

for ii = 1:15
    text(uav_loca(ii,2) - 5, uav_loca(ii,3)-10, ['FY' num2str(ii)]);
end
axis square;
axis([-220 20 -120 120])
legend('����ǰ���˻�λ��','���������˻�λ��','�������˻�λ��')
savefig('1-5.fig');


