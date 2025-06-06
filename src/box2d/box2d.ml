include Types
include Functions

module World = struct
  include Types.World
  include Functions.World
end

module Body = struct
  include Types.Body
  include Functions.Body
end

module Filter = struct
  include Types.Filter
end

module Shape = struct
  include Types.Shape
  include Functions.Shape
end
