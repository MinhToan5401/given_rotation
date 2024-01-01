function varargout = xoay_anh_given(varargin)

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @xoay_anh_given_OpeningFcn, ...
                   'gui_OutputFcn',  @xoay_anh_given_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before xoay_anh_given is made visible.
function xoay_anh_given_OpeningFcn(hObject, eventdata, handles, varargin)
guidata(hObject, handles);



% --- Outputs from this function are returned to the command line.
function varargout = xoay_anh_given_OutputFcn(hObject, eventdata, handles) 

% varargout{1} = handles.output;


function edit1_Callback(hObject, eventdata, handles)

function edit1_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)

% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
chon = get(handles.chon, 'value'); %khi chọn option trên slidebar 'lựa chọn số chiều' thì dòng này sẽ hiểu là mình chọn option nào. VD: Ảnh 2D, 3D hay 4D
switch chon %cấu trúc switch case, tức nếu mình chọn 2d nó sẽ chạy chương trình trong phạm vi case1, khi chọn 3d thì nó sẽ chạy chương trình trong phạm vi case 2...
    case 1 % Khi chọn ảnh 2D thì trong nguyên khối switch nó chỉ chạy những câu lệnh trong case 1
        set(handles.loaianh, 'string', 'grayscale'); %hiển thị ra ô 'loại ảnh' ngoài giao diện là grayscale
        axes(handles.axes1);%Từ dòng này trở xuông thì khi dùng hàm imshow để hiển thị ảnh thì nó sẽ hiển thị ở trục tọa độ axes1 ngoài giao diện
        [filename path] = uigetfile({'*.jpg'; '*.png'; '*.bmp'; '*.*'},'file selector');%tạo một cửa sổ và chỉ chọn những file đã được quy định trong dòng code này: jpg; png; bmp. Dòng code này đồng thời sẽ lưu tên file vào filename và lưu đường dẫn đến tên file vào path
        full = strcat (path, filename); %nối chuỗi, đơn giản là nối đường dẫn với file name để nó thành một cái link hoàn chỉnh dẫn đễn địa chỉ hình ảnh
        ImageData1 = imread(full);%hàm imread dùng để đọc hình ảnh, tham số truyền vào là địa chỉ đẫn đến ảnh đó đã được mình tạo ở trên khi thao tác chọn file 
        ImageData = rgb2gray(ImageData1);%chuyển ảnh đã chọn thành ảnh xám (grayscale)
        thongTinAnh = imfinfo(full);%lấy thông tin hình ảnh mình đã chọn
        set(handles.chieudaianh, 'string', thongTinAnh.Width);%xuất ra giao diện thông tin chiều dài ảnh
        set(handles.chieucaoanh, 'string', thongTinAnh.Height);%xuất ra giao diện thông tin chiều cao ảnh
        set(handles.BitDepth, 'string', thongTinAnh.BitDepth);%xuất ra giao diện thông bitdepth dài ảnh
        set(handles.Format, 'string', thongTinAnh.Format);%xuất ra giao diện thông tin định dạng ảnh

        imshow(ImageData);%dòng này để show bức ảnh gốc ra trục tọa độ 1 trên giao diện
        
        axes(handles.axes2);%Từ dòng này trở xuông thì khi dùng hàm imshow để hiển thị ảnh thì nó sẽ hiển thị ở trục tọa độ axes2 ngoài giao diện
        % lấy giá trị góc mình đã nhập ngoài giao diện
        gocxoay = get(handles.edit1, 'string');
        gocxoay = str2num(gocxoay);%giá trị góc mình lấy được thuộc kiểu chuỗi (string) mình phải chuyển nó sang kiểu số để xử lý
        gocxoay_rad = deg2rad(gocxoay);%sau khi có giá trị kiểu số của góc, mình quy đổi nó từ giá trị độ ra giá trị radian
        
        % Tạo ma trận xoay 2D
        CMatrix = [+cos(gocxoay_rad) +sin(gocxoay_rad); -sin(gocxoay_rad) +cos(gocxoay_rad)];
        
        
        % Tính toán kích thước của hình ảnh
        [X,Y,Z] = size(ImageData);%lấy kích thước ở từng chiều của hình ảnh
        set(handles.NumberOfChanels, 'string', Z);%đưa ra ngoài giao diện thông tin số kênh màu (Z là chiều thứ 3 của hình ảnh)
        
        Temp = round( [1 1; 1 Y; X 1; X Y]*CMatrix );
        %Temp cos - sin     sin + cos
        %     cos - sin   Y(sin + cos)
        %    X(cos - sin)   sin + cos
        %    X(cos - sin) Y(sin + cos)
                
        Temp = bsxfun(@minus, Temp, min(Temp)) + 1;
        %Ví dụ cho cách xài hàm trên 
        %a = [3 4 5; 1 2 3]
        %a = bsxfun(@minus, a, min(a)) + 1;
        %a = [3 3 3; 1 1 1]
        
        %Tạo một ma trận toàn số 0 để lưu thông tin ảnh sau khi xoay (ma trận toàn số 0 thì khi mình nhìn hình ảnh nó sẽ là full đen, ma trận full số 255 thì khi nhìn nó sẽ là full trắng, vì vậy khi xoay ảnh có những góc ảnh nó bị đen thì nguyên nhân là từ đây)
        OutputImage = zeros([max(Temp) Z],class(ImageData)); 
        
        % Implementation of rotaton function.
        for a = 1:size(OutputImage,1)%quét số hàng của ma trận ảnh output
        
            for b = 1:size(OutputImage,2)%quét số cột của ma trận ảnh output
                Rotation = ([a b]-Temp(1,:))*CMatrix.'; %tạo ma trận 1x2, CMatrix.' là ma trận chuyển vị của CMatrix. Dòng này để xử lý xoay ảnh
        
                if all(Rotation >= 1) && all(Rotation <= [X Y])%Câu lệnh này trả về giá trị đúng nếu cả Rotation[1] và Rotation[2] < 1 và all(Rotation <= [X Y]) trả về 1 nếu Rotation[1]<X và Rotation[2]<Y.
                    CL = ceil(Rotation);%làm tròn lên mảng rotation
        
                    FL = floor(Rotation);%làm tròn xuống mảng rotation
        
                    A = [...
                        ((CL(2)-Rotation(2))*(CL(1)-Rotation(1))),...
                        ((Rotation(2)-FL(2))*(Rotation(1)-FL(1)));
        
                        ((CL(2)-Rotation(2))*(Rotation(1)-FL(1))),...
                        ((Rotation(2)-FL(2))*(CL(1)-Rotation(1)))];
        
                    Color = bsxfun(@times, A, double(ImageData(FL(1):CL(1),FL(2):CL(2),:)));
        
                    OutputImage(a,b,:) = sum(sum(Color),2);%xử lý màu các điểm ảnh sau khi xoay để đảm bảo ảnh k bị mất chi tiết
                end
        
            end
        
        end        
        
        %Hiển thị ảnh đầu ra sau khi xoay
        imshow(OutputImage);
        
        %xuất kết quả góc xoay ra giao diện
        gocxoay = num2str(gocxoay);
        set(handles.gocxoay, 'string', gocxoay)

    case 2
        set(handles.loaianh, 'string', 'RGB 3 kênh màu');%xuất ra giao diện loại ảnh 'RGB 3 kênh màu'
        axes(handles.axes1);%Từ dòng này trở xuông thì khi dùng hàm imshow để hiển thị ảnh thì nó sẽ hiển thị ở trục tọa độ axes1 ngoài giao diện
        [filename path] = uigetfile({'*.jpg'; '*.png'; '*.bmp'; '*.*'},'file selector'); %tạo một cửa sổ và chỉ chọn những file đã được quy định trong dòng code này: jpg; png; bmp. Dòng code này đồng thời sẽ lưu tên file vào filename và lưu đường dẫn đến tên file vào path
        full = strcat (path, filename); %nối chuỗi, đơn giản là nối đường dẫn với file name để nó thành một cái link hoàn chỉnh dẫn đễn địa chỉ hình ảnh
        ImageData = imread(full);%hàm imread dùng để đọc hình ảnh, tham số truyền vào là địa chỉ đẫn đến ảnh đó đã được mình tạo ở trên khi thao tác chọn file
        thongTinAnh = imfinfo(full);%lấy thông tin ảnh
        %xuất các thông tin ảnh ra giao diện tương tự như ở case 1
        set(handles.chieudaianh, 'string', thongTinAnh.Width);
        set(handles.chieucaoanh, 'string', thongTinAnh.Height);
        set(handles.BitDepth, 'string', thongTinAnh.BitDepth);
        set(handles.Format, 'string', thongTinAnh.Format);

        imshow(ImageData);%Show ảnh ra giao diện

        axes(handles.axes2);%Từ dòng này trở xuông thì khi dùng hàm imshow để hiển thị ảnh thì nó sẽ hiển thị ở trục tọa độ axes2 ngoài giao diện
        %Lấy giá trị góc xoay từ giao diện
        gocxoay = get(handles.edit1, 'string');
        gocxoay = str2num(gocxoay);%chuyển giá trị góc xoay từ kiểu chuỗi sang kiểu số
        gocxoay_rad = deg2rad(gocxoay);%chuyển từ giá trị độ sang giá trị radian

        %Tạo ma trận xoay 
        CMatrix = [+cos(gocxoay_rad) +sin(gocxoay_rad); -sin(gocxoay_rad) +cos(gocxoay_rad)];

        %Tính toán kích thước theo từng chiều của ảnh
        [X,Y,Z] = size(ImageData);
        set(handles.NumberOfChanels, 'string', Z);%xuất ra giao diện số kênh màu

        Temp = round( [1 1; 1 Y; X 1; X Y]*CMatrix ); %temp 4x2

        %Temp cos - sin     sin + cos
        %     cos - sin   Y(sin + cos)
        %    X(cos - sin)   sin + cos
        %    X(cos - sin) Y(sin + cos)

        Temp = bsxfun(@minus, Temp, min(Temp)) + 1; % trừ các phần tử theo từng cột trong ma trận Temp cho số nhỏ nhất từng cột trong ma trận Temp rồi cộng thêm 1 để số nhỏ nhất bây giờ trở thành 1 
        
        %Ví dụ cho cách xài hàm trên 
        %a = [3 4 5; 1 2 3]
        %a = bsxfun(@minus, a, min(a)) + 1;
        %a = [3 3 3; 1 1 1]

        OutputImage = zeros([max(Temp) Z],class(ImageData)); %Tạo một ma trận toàn số 0 với kích thước [max(Temp) Z] với kiểu dữ liệu giống ImageData, %max(Temp) trả về hàng có chứa giá trị lớn nhất trong mảng

        % Implementation of rotaton function.
        for a = 1:size(OutputImage,1)

            for b = 1:size(OutputImage,2)
                Rotation = ([a b]-Temp(1,:))*CMatrix.'; %tạo ma trận 1x2, CMatrix.' là ma trận chuyển vị của CMatrix

                if all(Rotation >= 1) && all(Rotation <= [X Y]) %Câu lệnh này trả về giá trị đúng nếu cả Rotation[1] và Rotation[2] < 1 và all(Rotation <= [X Y]) trả về 1 nếu Rotation[1]<X và Rotation[2]<Y.
                    CL = ceil(Rotation);%làm tròn lên các giá trị trong mảng rotation

                    FL = floor(Rotation);%làm tròn xuống các giá trị trong mảng rotation

                    A = [...
                        ((CL(2)-Rotation(2))*(CL(1)-Rotation(1))),...
                        ((Rotation(2)-FL(2))*(Rotation(1)-FL(1)));

                        ((CL(2)-Rotation(2))*(Rotation(1)-FL(1))),...
                        ((Rotation(2)-FL(2))*(CL(1)-Rotation(1)))];

                    Color = bsxfun(@times, A, double(ImageData(FL(1):CL(1),FL(2):CL(2),:)));

                    OutputImage(a,b,:) = sum(sum(Color),2);%xử lý giá trị màu của các điểm ảnh
                end

            end

        end        

        %hiển thị ảnh ra axes2
        imshow(OutputImage);

        %xuất kết quả góc xoay
        gocxoay = num2str(gocxoay);
        set(handles.gocxoay, 'string', gocxoay); 

    case 3
        set(handles.loaianh, 'string', 'ảnh chuyển động gif');%xuất ra giao diện loại ảnh gif chuyển động
        axes(handles.axes1);%từ dòng này trở xuống khi sử dụng hàm imshow thì ảnh sẽ hiện thị trong axes1
        [filename path] = uigetfile({'*.gif'},'file selector'); %tạo một cửa sổ và chỉ chọn những file đã được quy định trong dòng code này: jpg; png; bmp. Dòng code này đồng thời sẽ lưu tên file vào filename và lưu đường dẫn đến tên file vào path
        full = strcat (path, filename); %nối chuỗi, đơn giản là nối đường dẫn với file name để nó thành một cái link hoàn chỉnh dẫn đễn địa chỉ hình ảnh
        ImageData = imread(full, 'Frames', 'all');%hàm imread dùng để đọc hình ảnh, tham số truyền vào là địa chỉ đẫn đến ảnh đó đã được mình tạo ở trên khi thao tác chọn file

        % lấy thông tin ảnh
        S = imfinfo(full);
        numImages = numel(S);%lấy số frame của ảnh

        % tạo hình ảnh trống
        hi = imshow(zeros(size(ImageData(:,:,1,1))));
        hi.CDataMapping = 'direct';
        
        %hiển thị ảnh gốc lên ảnh trống vừa tạo, cho vòng lặp để chạy từng frame ảnh
        for k = 1 : numImages
            hi.CData = ImageData(:,:,:,k);
            colormap(S(k).ColorTable) %Đặt bản đồ màu của hình thành bản đồ màu được chỉ định trong trường ColorTable của phần tử thứ k trong mảng cấu trúc S.
            
            caption = sprintf('Frame %#d of %d', k, numImages); %hiện thị ra  giao diện số thứ tự frame đang chạy từ đầu đến hết
            title(caption);
            drawnow;
            
            % tốc số hiển thị giữa mỗi frame là 0.01s
            pause(0.01)
        end
        axes(handles.axes2);
        % Take the input of the angle from user.
        gocxoay = get(handles.edit1, 'string');
        gocxoay = str2num(gocxoay);

        % tạo một ảnh trống để lưu ảnh sau khi xoay
        hi = imshow(zeros(size(ImageData(:,:,1,1))));
        hi.CDataMapping = 'direct';

        %Tiến hành xoay và hiển thị ảnh đã xoay
        for i = 1 : numImages        
            % Lấy frame thứ k
            current_frame = ImageData(:, :, :, i);

            % Xoay frame thứ k
            rotated_frame = imrotate(current_frame, gocxoay, 'nearest', 'crop');

            % Lưu frame đã xoay vào mảng mới
            rotated_gifData(:, :, :, i) = rotated_frame;

            hi.CData = rotated_gifData(:, :, :,i);
            colormap(S(i).ColorTable)

            caption = sprintf('Frame %#d of %d', i, numImages);
            title(caption);
            drawnow;

            % tốc số hiển thị giữa mỗi frame là 0.01s
            pause(0.01)
        end

        %xuất kết quả góc xoay
        gocxoay = num2str(gocxoay);
        set(handles.gocxoay, 'string', gocxoay); 

    case 4
        set(handles.loaianh, 'string', 'Ảnh 3D y tế');%Hiển thị loại ảnh là 'Ảnh 3D y tế'
        axes(handles.axes1);%Từ dòng này trở đi khi sử dụng imshow thì hình ảnh sẽ hiển thị ra trục tọa độ 1
        [filename, path] = uigetfile({'*.nii'},'file selector'); %coi lại case 1
        full = strcat (path, filename); %coi lại case 1
        niiInfo = niftiinfo(full); %Hàm này lấy thông tin ảnh y tế đuôi .nii
        vol = niftiread(full); %đọc ảnh đuôi .nii

        % hiển thị hình ảnh 3D thì mình k dùng imshow mà mình dùng
        % volshowvolshow (image là ảnh 2d thì có pixel, volumn là ảnh 3d thì có volxel)
        volshow(vol);

        % lấy thông tin góc người dùng đã nhập trong giao diện
        gocxoay = get(handles.edit1, 'string');
        gocxoay = str2num(gocxoay);
        gocxoay_rad = deg2rad(gocxoay);
        chieu = get(handles.chieu, 'value');
        set(handles.sizeanh, 'string', niiInfo.ImageSize);
        set(handles.Format, 'string', 'nii');
        
        %sử dụng cấu trúc điều kiện switch case, nếu chọn trên slidebar
        %quay theo trục X thì là case 1, trục y là case 2, trục z là case 3
        switch chieu
            case 1
                volRot = imrotate3(vol, gocxoay, [1, 0, 0], 'nearest', 'loose');%xoay ảnh theo hàm đã có sẵn trong matlab
            case 2
                volRot = imrotate3(vol, gocxoay, [0, 1, 0], 'nearest', 'loose');
            case 3
                volRot = imrotate3(vol, gocxoay, [0, 0, 1], 'nearest', 'loose');
        end
        % hiển thị hình ảnh sau khi xoay
        volshow(volRot);
        %xuất kết quả góc xoay
        gocxoay = num2str(gocxoay);
        set(handles.gocxoay, 'string', gocxoay); 
    case 5
        axes(handles.axes1);
        set(handles.loaianh, 'string', 'ảnh 3D');
        [filename path] = uigetfile({'*.stl'},'file selector'); 
        full = strcat (path, filename); %nối chuỗi
        model = stlread(full);
        %plot the origin object
        h1 = trimesh(model);
        view(handles.axes1);
        %make point and conectivity matrice from this model
        points = model.Points;
        cList = model.ConnectivityList;
        
        axes(handles.axes2);
        % Take the input of the angle from user.   
        gocxoay = get(handles.edit1, 'string');
        gocxoay = str2num(gocxoay);
        gocxoay_rad = deg2rad(gocxoay);

        thetax = get(handles.edit2, 'string');
        thetax = str2num(thetax);
        thetax_rad = deg2rad(thetax);

        thetay = get(handles.edit3, 'string');
        thetay = str2num(thetay);
        thetay_rad = deg2rad(thetay);

        thetaz = get(handles.edit4, 'string');
        thetaz = str2num(thetaz);
        thetaz_rad = deg2rad(thetaz);
        %rotation matrix
        Rx = [1 0 0; 0 cos(gocxoay_rad) -sin(gocxoay_rad); 0 sin(gocxoay_rad) cos(gocxoay_rad)];
        Ry = [cos(gocxoay_rad) 0 sin(gocxoay_rad); 0 1 0; -sin(gocxoay_rad) 0 cos(gocxoay_rad)];
        Rz = [cos(gocxoay_rad) -sin(gocxoay_rad) 0; sin(gocxoay_rad) cos(gocxoay_rad) 0; 0 0 1];

        RX = [1 0 0; 0 cos(thetax_rad) -sin(thetax_rad); 0 sin(thetax_rad) cos(thetax_rad)];
        RY = [cos(thetay_rad) 0 sin(thetay_rad); 0 1 0; -sin(thetay_rad) 0 cos(thetay_rad)];
        RZ = [cos(thetaz_rad) -sin(thetaz_rad) 0; sin(thetaz_rad) cos(thetaz_rad) 0; 0 0 1];
        
        chieu = get(handles.chieu, 'value');
        switch chieu
            case 1                
                pointR = points*Rx;
            case 2
                pointR = points*Ry;
            case 3
                pointR = points*Rz;
            case 4
                pointR = points*RX;
                pointR = points*RY;
                pointR = points*RZ;
        end  

        %plot the rotated object
        h2 = trimesh(cList, pointR(:,1),pointR(:,2),pointR(:,3)); 
        view(handles.axes2);

        %xuất kết quả góc xoay
        gocxoay = num2str(gocxoay);
        set(handles.gocxoay, 'string', gocxoay); 
end 


% --- Executes on selection change in chon.
function chon_Callback(hObject, eventdata, handles)

function chon_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in chieu.
function chieu_Callback(hObject, eventdata, handles)
% hObject    handle to chieu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns chieu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from chieu


% --- Executes during object creation, after setting all properties.
function chieu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to chieu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
