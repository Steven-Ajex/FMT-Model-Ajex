function export_each_struct_to_csv(outputDir)
% 将当前工作区中所有 struct（字段为 timeseries）导出为独立 CSV
% 自动处理：时间类型转换、数值类型统一、字段名冲突修复

    if nargin < 1 || isempty(outputDir)
        outputDir = fullfile(pwd, 'csv_out');
    end
    if ~exist(outputDir, 'dir'); mkdir(outputDir); end

    ws = evalin('base', 'whos');

    for k = 1:numel(ws)
        name = ws(k).name;
        if ~strcmp(ws(k).class, 'struct')
            continue;
        end

        S = evalin('base', name);
        fns = fieldnames(S);
        tsFields = fns(structfun(@(x) isa(x, 'timeseries'), S));
        if isempty(tsFields)
            continue;
        end

        TT = [];
        for i = 1:numel(tsFields)
            ts = S.(tsFields{i});

            % ---- 处理时间轴 ----
            t = ts.Time;
            if isa(t, 'datetime')
                tvec = t;
            elseif isa(t, 'duration')
                tvec = t;
            elseif isnumeric(t)
                tvec = seconds(t); % 假定单位为秒
            else
                warning('? 无法识别时间类型：%s.%s，跳过。', name, tsFields{i});
                continue;
            end

            % ---- 处理数据 ----
            data = ts.Data;
            if ~isfloat(data)
                data = double(data); % 统一类型
            end
            if ismatrix(data) && size(data,1) < size(data,2)
                data = data.';
            end

            varNames = composeVarNames(tsFields{i}, size(data,2));

            try
                tt = timetable(tvec, data, 'VariableNames', varNames);
            catch ME
                warning('? 构建 timetable 失败: %s.%s (%s)', name, tsFields{i}, ME.message);
                continue;
            end

            if isempty(TT)
                TT = tt;
            else
                TT = synchronize(TT, tt, 'union', 'linear');
            end
        end

        if isempty(TT)
            warning('? %s 无有效 timeseries，跳过。', name);
            continue;
        end

        % ---- 修复时间列名称冲突 ----
        T = timetable2table(TT, 'ConvertRowTimes', true);
        existingNames = T.Properties.VariableNames;

        if any(strcmp(existingNames, 'timestamp'))
            % 如果已经存在 timestamp 字段，则时间列改名为 time
            T.Properties.VariableNames{1} = 'time';
        else
            T.Properties.VariableNames{1} = 'timestamp';
        end

        % ---- 导出 CSV ----
        outPath = fullfile(outputDir, [name '.csv']);
        writetable(T, outPath);
        fprintf('? 导出 %s（%d 行，%d 列）\n', outPath, size(T,1), size(T,2));
    end
end

function names = composeVarNames(baseName, ncol)
% 自动生成变量名（支持多列）
    if ncol == 1
        names = {baseName};
    else
        names = arrayfun(@(i) sprintf('%s_%d', baseName, i), 1:ncol, 'UniformOutput', false);
    end
end
