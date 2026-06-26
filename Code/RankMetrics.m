% Snippet to add a single 'Score' column to metrics_table.
% Counts, per test and technique, how many metrics (from rankMetrics) it ranked 1st in.

% Define the metrics to consider for first-place counts (all error-based, lower is better)
rankMetrics = {'RMSE_Pos', 'RMSE_Att', 'SMAPE', 'ISE', 'IAE', 'ITAE', 'ITSE'};

% New column Score already added in metrics_table

% Get unique tests
uniqueTests = unique(metrics_table.Test);

% For each test, compute first-place counts per technique
for testnum = 1:length(uniqueTests)
    test = uniqueTests{testnum};
    testRows = strcmp(metrics_table.Test, test);  % Logical index for rows of this test
    numTechs = sum(testRows);

    firstPlaces = zeros(numTechs, 1);  % Temp array for this test's techniques

    for m = 1:length(rankMetrics)
        metric = rankMetrics{m};
        values = metrics_table.(metric)(testRows);  % Extract values for this test/metric

        [~, sortIdx] = sort(values, 'ascend');  % Sort ascending (lowest error best)

        % Find the index(es) with the minimum value (handles ties)
        minVal = values(sortIdx(1));
        bestIdx = find(values == minVal);  % All tied for first

        % Increment count for each best technique
        % Note: For ties, this gives full credit to all tied.
        firstPlaces(bestIdx) = firstPlaces(bestIdx) + 1;
    end

    % Assigning back to table
    metrics_table.Score(testRows) = firstPlaces;
end

% Sorting table by Test then by Score descending for easier viewing
metrics_table = sortrows(metrics_table, {'Test', 'Score'}, {'ascend', 'descend'});
